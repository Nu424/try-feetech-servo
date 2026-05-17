"""STS シリーズ シリアルバスサーボ コントローラ。

公式の Feetech SDK (`scservo_sdk`) をベースに、STS3215 などの STS系
サーボを扱いやすくラップしたコントローラを提供する。

主な特徴:
    - モード遷移時のEEPROM書き込みを最小化（前回モードをキャッシュ）
    - 速度・加速度・PWMを 0〜1 / -1〜+1 の比率で正規化
    - ソフトリミット、コンテキストマネージャ、専用例外を内蔵
    - 単一/複数の完了待ち、状態取得（温度・電圧・負荷など）に対応

依存:
    pip install ftservo-python-sdk
    （または scservo_sdk が import 可能な任意のFeetech公式由来SDK）

使用例:
    >>> from sts_controller import STSController
    >>> with STSController(port="COM4", ids=[1, 2]) as ctrl:
    ...     ctrl.move_to_angle(1, 180.0, speed_ratio=0.5, acc_ratio=0.3)
    ...     ctrl.wait_until_done(1)
    ...     print(ctrl.get_position(1))

注意:
    このクラスはスレッドセーフではない。並行アクセスする場合は
    呼び出し側で排他制御すること。
"""

from __future__ import annotations

import time
from typing import Optional

from scservo_sdk import (
    COMM_SUCCESS,
    GroupSyncWrite,
    PortHandler,
    sms_sts,
)


# ============================================================
# 例外クラス
# ============================================================


class FeetechError(Exception):
    """このラッパー全体の基底例外。"""


class CommError(FeetechError):
    """通信エラー（パケット消失、CRC不一致、タイムアウトなど）。"""


class SoftLimitError(FeetechError):
    """ソフトリミット範囲外の指令。"""


class NotConnectedError(FeetechError):
    """接続前に通信メソッドが呼ばれた。"""


# ============================================================
# 内部定数（STSレジスタアドレス・モード番号）
# ============================================================

# --- EEPROM領域 ---
_ADDR_OPERATING_MODE = 33  # 0x21  動作モード（0:位置 / 1:速度 / 2:PWM / 3:ステップ）

# --- RAM領域 ---
_ADDR_TORQUE_ENABLE = 40  # 0x28  トルク有効/無効
_ADDR_GOAL_ACC = 41  # 0x29  加速度（SyncWritePosExの先頭）
_ADDR_GOAL_POSITION = 42  # 0x2A  目標位置 (2 byte)
_ADDR_RUNNING_TIME = 44  # 0x2C  Running Time / PWMモード時はPWM値 (2 byte)
_ADDR_RUNNING_SPEED = 46  # 0x2E  目標速度 (2 byte)
_ADDR_PRESENT_POSITION = 56  # 0x38  現在位置 (2 byte)
_ADDR_PRESENT_SPEED = 58  # 0x3A  現在速度 (2 byte, signed)
_ADDR_PRESENT_LOAD = 60  # 0x3C  現在負荷 (2 byte, signed)
_ADDR_PRESENT_VOLTAGE = 62  # 0x3E  電圧（×0.1V）
_ADDR_PRESENT_TEMPERATURE = 63  # 0x3F  温度（℃）
_ADDR_MOVING = 66  # 0x42  動作中フラグ（1:動作中 / 0:停止）

# --- 動作モード番号 ---
_MODE_POSITION = 0  # 絶対角度制御
_MODE_WHEEL = 1  # 連続回転（速度制御）
_MODE_PWM = 2  # PWM 開ループ
_MODE_STEP = 3  # 相対ステップ（このラッパーでは未使用）

# --- SyncWritePosEx 用 ---
_SYNC_DATA_LEN = 7  # ACC(1) + POS(2) + TIME(2) + SPEED(2) = 7 bytes


# ============================================================
# 内部ユーティリティ
# ============================================================


def _clamp(x: float, lo: float, hi: float) -> float:
    """値を指定範囲にクランプする。

    Args:
        x: クランプ対象の値。
        lo: 下限。
        hi: 上限。

    Returns:
        [lo, hi] に丸めた値。
    """
    return max(lo, min(hi, x))


def _signed_to_register(value: int, direction_bit: int) -> int:
    """符号付き整数を「方向ビット + 絶対値」のレジスタ生値へエンコードする。

    Feetech サーボでは PWMモードの speed や ステップモードの位置で、
    指定ビットに方向情報を入れる慣習がある。

    Args:
        value: 符号付き値。
        direction_bit: 方向を表すビット位置（例: PWM時は10、Wheel時は15）。

    Returns:
        レジスタへ書き込む符号なし生値。
    """
    magnitude_mask = (1 << direction_bit) - 1
    if value < 0:
        return (1 << direction_bit) | (abs(value) & magnitude_mask)
    return value & magnitude_mask


# ============================================================
# メインクラス
# ============================================================


class STSController:
    """Feetech STS シリーズ サーボのコントローラ。

    内部モード管理・ratio正規化・ソフトリミットを備えた高レベルAPI。
    コンテキストマネージャとして使うのを推奨（with句を抜けると自動で
    トルクオフ＋切断される）。

    Attributes:
        port (str): シリアルポート名（例: "COM4"）。
        ids (list[int]): 管理対象のサーボIDリスト。
        steps_per_rev (int): 1回転あたりのエンコーダステップ数。
        max_speed (int): 速度レジスタの最大値（ratio=1のとき書き込まれる）。
        max_acc (int): 加速度レジスタの最大値（ratio=1のとき書き込まれる）。
        baudrate (int): 通信ボーレート。
        angle_limits (dict[int, tuple[float, float]]): IDごとのソフトリミット。
    """

    # ------------------------------------------------------------
    # 初期化
    # ------------------------------------------------------------

    def __init__(
        self,
        port: str,
        ids: list[int],
        steps_per_rev: int = 4095,
        max_speed: int = 32767,
        max_acc: int = 255,
        baudrate: int = 1_000_000,
        angle_limits: Optional[dict[int, tuple[float, float]]] = None,
    ):
        """コントローラを構築する（まだ接続はしない）。

        Args:
            port: シリアルポート名（Windowsなら "COM4" など）。
            ids: 管理するサーボIDのリスト。
            steps_per_rev: 1回転あたりのステップ数。STS3215のデフォルトは4095。
            max_speed: speed_ratio=1.0 のときに書き込む速度生値。SDK上限は32767。
            max_acc: acc_ratio=1.0 のときに書き込む加速度生値。SDK上限は255。
            baudrate: 通信ボーレート。STS3215のデフォルトは1,000,000。
            angle_limits: IDごとのソフトリミット {id: (min_deg, max_deg)}。
                範囲外の move_to_angle 呼び出しは SoftLimitError を投げる。

        Raises:
            ValueError: ids が空、または数値パラメータが不正な場合。
        """
        # --- 入力検証 ---
        if not ids:
            raise ValueError("ids が空です")
        if steps_per_rev <= 0:
            raise ValueError("steps_per_rev は正の整数で")
        if max_speed <= 0 or max_acc <= 0:
            raise ValueError("max_speed / max_acc は正の値で")

        # --- 設定値の保持 ---
        self.port = port
        self.ids = list(ids)
        self.steps_per_rev = steps_per_rev
        self.max_speed = max_speed
        self.max_acc = max_acc
        self.baudrate = baudrate
        self.angle_limits = dict(angle_limits) if angle_limits else {}

        # --- 内部状態 ---
        self._port_handler: Optional[PortHandler] = None
        self._packet: Optional[sms_sts] = None
        self._mode_cache: dict[int, int] = {}  # id -> 最後に書いたモード
        self._connected: bool = False

    # ------------------------------------------------------------
    # 接続・切断
    # ------------------------------------------------------------

    def connect(self) -> None:
        """ポートをオープンし、全サーボの生存確認とモードキャッシュ初期化を行う。

        実機の現在モードを必ず一度読み出してキャッシュに反映するため、
        前回プロセスがどのモードで終了していても整合性が保たれる。

        Raises:
            CommError: ポートオープン、ボーレート設定、いずれかのIDへの
                ping、またはモード読み取りに失敗した場合。
        """
        # --- ポートオープン ---
        self._port_handler = PortHandler(self.port)
        if not self._port_handler.openPort():
            raise CommError(f"ポート {self.port} を開けません")
        if not self._port_handler.setBaudRate(self.baudrate):
            self._port_handler.closePort()
            raise CommError(f"ボーレート {self.baudrate} を設定できません")

        self._packet = sms_sts(self._port_handler)

        # --- 全サーボ生存確認 ＋ モードキャッシュ同期 ---
        for sid in self.ids:
            if not self._ping_raw(sid):
                self._port_handler.closePort()
                self._port_handler = None
                self._packet = None
                raise CommError(f"ID {sid} に応答なし（配線・電源・IDを確認）")
            # 実機モードを読んでキャッシュに反映（書き込みは発生しない）
            self._mode_cache[sid] = self._read_mode_raw(sid)

        self._connected = True

    def disconnect(self) -> None:
        """全サーボのトルクをオフにしてポートを閉じる。

        できる限りクリーンに終了することを優先し、個別の例外は飲み込む。
        重複呼び出しは安全（無視）。
        """
        if not self._connected:
            return

        # --- 全軸トルクオフ（個別失敗は黙殺） ---
        for sid in self.ids:
            try:
                self.disable_torque(sid)
            except FeetechError:
                pass

        # --- ポートクローズ ---
        try:
            if self._port_handler is not None:
                self._port_handler.closePort()
        finally:
            self._connected = False
            self._port_handler = None
            self._packet = None

    def __enter__(self) -> "STSController":
        """with文で使えるよう、入場時に接続する。

        Returns:
            自身。
        """
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """with文の退出時に切断する。例外は伝播させる。"""
        self.disconnect()

    # ------------------------------------------------------------
    # モード管理（内部）
    # ------------------------------------------------------------

    def _read_mode_raw(self, sid: int) -> int:
        """実機の現在モードを読む（キャッシュ無視・内部用）。

        Args:
            sid: サーボID。

        Returns:
            モード番号（0〜3）。

        Raises:
            CommError: 通信失敗時。
        """
        val, comm, err = self._packet.read1ByteTxRx(sid, _ADDR_OPERATING_MODE)
        self._check_result(sid, "モード読み取り", comm, err)
        return val

    def _set_mode_raw(self, sid: int, mode: int) -> None:
        """EEPROM ロック解除→モード書込→再ロックの一連を実行する。

        モード切替後はサーボ内部の状態が落ち着くまで少し待つ。

        Args:
            sid: サーボID。
            mode: 設定するモード番号（0〜3）。

        Raises:
            CommError: 書込失敗時。
        """
        comm, err = self._packet.unLockEprom(sid)
        self._check_result(sid, "EEPROMロック解除", comm, err)
        write_succeeded = False
        try:
            comm, err = self._write_1byte_raw(sid, _ADDR_OPERATING_MODE, mode)
            self._check_result(sid, "モード書込", comm, err)
            write_succeeded = True
        finally:
            lock_comm, lock_err = self._packet.LockEprom(sid)
            if write_succeeded:
                self._check_result(sid, "EEPROM再ロック", lock_comm, lock_err)
        # モード切替直後の挙動安定化のため軽くスリープ
        time.sleep(0.02)

    def _ensure_mode(self, sid: int, mode: int) -> None:
        """キャッシュと異なるときだけモード書込を行う（EEPROM寿命対策）。

        Args:
            sid: サーボID。
            mode: 期待するモード番号。
        """
        if self._mode_cache.get(sid) != mode:
            self._set_mode_raw(sid, mode)
            self._mode_cache[sid] = mode

    # ------------------------------------------------------------
    # 内部ガード
    # ------------------------------------------------------------

    def _require_connected(self) -> None:
        """未接続なら NotConnectedError を投げる。

        Raises:
            NotConnectedError: connect() 未呼び出しの場合。
        """
        if not self._connected or self._packet is None:
            raise NotConnectedError(
                "未接続です。connect() を呼ぶか with 文で使ってください"
            )

    def _require_id(self, sid: int) -> None:
        """対象IDが登録済みか確認する。

        Args:
            sid: サーボID。

        Raises:
            ValueError: 未登録のIDが渡された場合。
        """
        if sid not in self.ids:
            raise ValueError(f"ID {sid} は登録されていません（登録済み: {self.ids}）")

    def _write_1byte_raw(self, sid: int, addr: int, value: int) -> tuple[int, int]:
        """SDKのバージョン差分を吸収して1バイト値を書き込む。"""
        write_fn = getattr(self._packet, "write1ByteTxRx", None)
        if write_fn is None:
            write_fn = getattr(self._packet, "writeByte", None)
        if write_fn is None:
            raise FeetechError("SDKに write1ByteTxRx / writeByte メソッドが見つかりません")
        return write_fn(sid, addr, value)

    def _check_result(self, sid: int, action: str, comm: int, err: int = 0) -> None:
        """通信結果とサーボ側エラーをまとめて検査する。"""
        if comm != COMM_SUCCESS:
            detail = self._packet.getTxRxResult(comm) if self._packet is not None else ""
            suffix = f" {detail}" if detail else ""
            raise CommError(f"ID {sid}: {action}失敗 (comm={comm}){suffix}")
        if err:
            detail = self._packet.getRxPacketError(err) if self._packet is not None else ""
            suffix = f" {detail}" if detail else ""
            raise CommError(f"ID {sid}: {action}サーボエラー (err={err}){suffix}")

    # ------------------------------------------------------------
    # 制御コマンド：モード0（角度指定）
    # ------------------------------------------------------------

    def move_to_angle(
        self,
        sid: int,
        angle_deg: float,
        speed_ratio: float = 0.5,
        acc_ratio: float = 0.5,
    ) -> None:
        """指定の角度（deg, 0〜360）へ動かす（モード0/位置制御）。

        speed_ratio・acc_ratio はどちらも生値の最低が 1 にクランプされる
        （SDK仕様で 0 は「制限なし＝最大」を意味するため、ratio=0 で
        瞬時最大動作になる事故を防ぐ）。

        Args:
            sid: サーボID。
            angle_deg: 目標角度 [deg]。0〜360 の範囲。
            speed_ratio: 速度比。0〜1。1で max_speed。
            acc_ratio: 加速度比。0〜1。1で max_acc。

        Raises:
            ValueError: angle_deg が 0〜360 の範囲外、または ID 未登録。
            SoftLimitError: angle_limits の範囲外を指令した。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        # --- 前提チェック ---
        self._require_connected()
        self._require_id(sid)
        if not 0.0 <= angle_deg <= 360.0:
            raise ValueError(f"angle_deg は 0〜360 で指定（与えられた値: {angle_deg}）")

        # --- ソフトリミットチェック ---
        if sid in self.angle_limits:
            lo, hi = self.angle_limits[sid]
            if not lo <= angle_deg <= hi:
                raise SoftLimitError(
                    f"ID {sid}: angle={angle_deg} はソフトリミット [{lo}, {hi}] の外"
                )

        # --- モード確認（必要なら切替） ---
        self._ensure_mode(sid, _MODE_POSITION)

        # --- 単位変換 ---
        position = int(round(angle_deg / 360.0 * self.steps_per_rev))
        speed_raw = max(1, int(_clamp(speed_ratio, 0.0, 1.0) * self.max_speed))
        acc_raw = max(1, int(_clamp(acc_ratio, 0.0, 1.0) * self.max_acc))

        # --- 送信 ---
        comm, err = self._packet.WritePosEx(sid, position, speed_raw, acc_raw)
        self._check_result(sid, "WritePosEx", comm, err)

    def move_to_angle_sync(
        self,
        targets: dict[int, float],
        speed_ratio: float = 0.5,
        acc_ratio: float = 0.5,
    ) -> None:
        """複数サーボへ角度指令を一括ブロードキャストする（SyncWrite）。

        個別の WritePosEx をループするより通信パケット数を大幅に削減できる。
        全IDがモード0でない場合は内部で個別にモード切替する。

        Args:
            targets: {id: angle_deg} の辞書。すべて 0〜360 [deg]。
            speed_ratio: 全軸共通の速度比。0〜1。
            acc_ratio: 全軸共通の加速度比。0〜1。

        Raises:
            ValueError: angle_deg が範囲外、または未登録IDを含む。
            SoftLimitError: ソフトリミット範囲外を含む。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        # --- 前提・バリデーション（全件先にチェック）---
        self._require_connected()
        if not targets:
            return
        for sid, ang in targets.items():
            self._require_id(sid)
            if not 0.0 <= ang <= 360.0:
                raise ValueError(f"ID {sid}: angle={ang} が 0〜360 外")
            if sid in self.angle_limits:
                lo, hi = self.angle_limits[sid]
                if not lo <= ang <= hi:
                    raise SoftLimitError(
                        f"ID {sid}: angle={ang} はソフトリミット [{lo}, {hi}] の外"
                    )

        # --- 全軸をモード0へ揃える ---
        for sid in targets:
            self._ensure_mode(sid, _MODE_POSITION)

        # --- 共通の生値 ---
        speed_raw = max(1, int(_clamp(speed_ratio, 0.0, 1.0) * self.max_speed))
        acc_raw = max(1, int(_clamp(acc_ratio, 0.0, 1.0) * self.max_acc))

        # --- GroupSyncWrite を組み立てて送信 ---
        gsw = GroupSyncWrite(
            self._packet,
            _ADDR_GOAL_ACC,
            _SYNC_DATA_LEN,
        )
        try:
            for sid, ang in targets.items():
                position = int(round(ang / 360.0 * self.steps_per_rev))
                # 7バイトのペイロード: ACC, POS_L, POS_H, TIME_L, TIME_H, SPEED_L, SPEED_H
                param = [
                    acc_raw & 0xFF,
                    position & 0xFF,
                    (position >> 8) & 0xFF,
                    0,
                    0,  # Running Time = 0
                    speed_raw & 0xFF,
                    (speed_raw >> 8) & 0xFF,
                ]
                if not gsw.addParam(sid, param):
                    raise CommError(f"ID {sid}: GroupSyncWrite.addParam 失敗")
            comm = gsw.txPacket()
            if comm != COMM_SUCCESS:
                detail = self._packet.getTxRxResult(comm)
                suffix = f" {detail}" if detail else ""
                raise CommError(f"GroupSyncWrite.txPacket 失敗 (comm={comm}){suffix}")
        finally:
            # 送信成功・失敗いずれも内部バッファをクリア
            gsw.clearParam()

    # ------------------------------------------------------------
    # 制御コマンド：モード1（連続回転・速度制御）
    # ------------------------------------------------------------

    def move_with_speed(
        self,
        sid: int,
        speed_ratio: float,
        acc_ratio: float = 0.5,
    ) -> None:
        """連続回転モードで速度指令を送る（モード1）。

        ratio 規約は他メソッドと統一されているが、本メソッドの speed_ratio
        のみ符号付き（-1〜+1）。符号で回転方向、絶対値で速さを表す。

        Args:
            sid: サーボID。
            speed_ratio: 速度比。-1〜+1。0で停止。
            acc_ratio: 加速度比。0〜1。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        # --- 前提 ---
        self._require_connected()
        self._require_id(sid)

        # --- モード確認 ---
        self._ensure_mode(sid, _MODE_WHEEL)

        # --- 単位変換（速度は符号付き、加速度は非負） ---
        speed_raw = int(_clamp(speed_ratio, -1.0, 1.0) * self.max_speed)
        acc_raw = max(1, int(_clamp(acc_ratio, 0.0, 1.0) * self.max_acc))

        # --- 送信（WriteSpec は SDK バージョンで名前揺れがあるので両対応）---
        write_fn = getattr(self._packet, "WriteSpec", None) or getattr(
            self._packet, "WriteSpe", None
        )
        if write_fn is None:
            raise FeetechError("SDKに WriteSpec / WriteSpe メソッドが見つかりません")
        comm, err = write_fn(sid, speed_raw, acc_raw)
        self._check_result(sid, "速度書込", comm, err)

    # ------------------------------------------------------------
    # 制御コマンド：モード2（PWM 開ループ）
    # ------------------------------------------------------------

    def move_with_pwm(self, sid: int, duty_ratio: float) -> None:
        """PWM開ループモードでデューティ指令を送る（モード2）。

        生値は -1000〜+1000 にマッピングされ、BIT10 が方向ビットとして
        セットされる。

        Args:
            sid: サーボID。
            duty_ratio: PWMデューティ比。-1〜+1。符号で方向、絶対値で強さ。
                0で停止。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        # --- 前提 ---
        self._require_connected()
        self._require_id(sid)

        # --- モード確認 ---
        self._ensure_mode(sid, _MODE_PWM)

        # --- ratio → 符号付き整数（-1000〜+1000） ---
        duty = _clamp(duty_ratio, -1.0, 1.0)
        signed_pwm = int(round(duty * 1000))
        signed_pwm = max(-1000, min(1000, signed_pwm))

        # --- BIT10 方向ビット + 下位10bit絶対値にエンコード ---
        word = _signed_to_register(signed_pwm, direction_bit=10)

        # --- 送信 ---
        comm, err = self._packet.write2ByteTxRx(sid, _ADDR_RUNNING_TIME, word)
        self._check_result(sid, "PWM書込", comm, err)

    # ------------------------------------------------------------
    # トルク ON / OFF
    # ------------------------------------------------------------

    def enable_torque(self, sid: int) -> None:
        """指定サーボのトルクをオンにする。

        Args:
            sid: サーボID。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)
        comm, err = self._write_1byte_raw(sid, _ADDR_TORQUE_ENABLE, 1)
        self._check_result(sid, "トルクON", comm, err)

    def disable_torque(self, sid: int) -> None:
        """指定サーボのトルクをオフにする（手で動かせる状態）。

        Args:
            sid: サーボID。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)
        comm, err = self._write_1byte_raw(sid, _ADDR_TORQUE_ENABLE, 0)
        self._check_result(sid, "トルクOFF", comm, err)

    # ------------------------------------------------------------
    # 状態取得
    # ------------------------------------------------------------

    def ping(self, sid: int) -> bool:
        """サーボが応答するかを確認する。

        Args:
            sid: サーボID。

        Returns:
            応答あり True / 応答なし False。

        Raises:
            NotConnectedError: 未接続。
        """
        self._require_connected()
        return self._ping_raw(sid)

    def _ping_raw(self, sid: int) -> bool:
        """ping の内部実装（例外を投げずに bool で返す）。"""
        _, comm, err = self._packet.ping(sid)
        return comm == COMM_SUCCESS and err == 0

    def get_position(self, sid: int) -> float:
        """現在位置を角度 [deg, 0〜360] で取得する。

        どのモード中でも呼び出し可能。連続回転モードでは1回転内の角度として
        ラップアラウンドされた値が返る点に注意。

        Args:
            sid: サーボID。

        Returns:
            現在角度 [deg]。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)
        pos, comm, err = self._packet.ReadPos(sid)
        self._check_result(sid, "位置読取", comm, err)
        # 0〜steps_per_rev → 0〜360 へ変換。範囲外は wrap してから返す
        return (pos % (self.steps_per_rev + 1)) / self.steps_per_rev * 360.0

    def get_speed(self, sid: int) -> int:
        """現在の回転速度を符号付き生値で取得する。

        Args:
            sid: サーボID。

        Returns:
            速度生値（正負で回転方向、絶対値で速さ）。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)
        spd, comm, err = self._packet.ReadSpeed(sid)
        self._check_result(sid, "速度読取", comm, err)
        return spd

    def get_load(self, sid: int) -> int:
        """現在の負荷を符号付き生値で取得する。

        およそ ±1000 が定格トルク 100% に相当。衝突検知や過負荷監視に使える。

        Args:
            sid: サーボID。

        Returns:
            負荷生値（符号付き）。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)
        raw, comm, err = self._packet.read2ByteTxRx(sid, _ADDR_PRESENT_LOAD)
        self._check_result(sid, "負荷読取", comm, err)
        return self._packet.scs_tohost(raw, 10)

    def get_temperature(self, sid: int) -> float:
        """サーボ内部温度を [℃] で取得する。

        STS3215 はおおむね 65℃ を超えるとアラーム→自動的にトルクが切れる。
        長時間運転時の監視推奨。

        Args:
            sid: サーボID。

        Returns:
            温度 [℃]。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)
        raw, comm, err = self._packet.read1ByteTxRx(sid, _ADDR_PRESENT_TEMPERATURE)
        self._check_result(sid, "温度読取", comm, err)
        return float(raw)

    def get_voltage(self, sid: int) -> float:
        """電源電圧を [V] で取得する。

        Args:
            sid: サーボID。

        Returns:
            電圧 [V]。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)
        raw, comm, err = self._packet.read1ByteTxRx(sid, _ADDR_PRESENT_VOLTAGE)
        self._check_result(sid, "電圧読取", comm, err)
        # 生値はおよそ「電圧×10」（70 = 7.0V）の慣習
        return raw * 0.1

    def get_mode(self, sid: int) -> int:
        """実機から現在の動作モードを読む（キャッシュも更新される）。

        Args:
            sid: サーボID。

        Returns:
            モード番号（0:位置 / 1:速度 / 2:PWM / 3:ステップ）。

        Raises:
            ValueError: ID未登録。
            CommError: 通信エラー。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)
        mode = self._read_mode_raw(sid)
        self._mode_cache[sid] = mode
        return mode

    # ------------------------------------------------------------
    # 完了待ち
    # ------------------------------------------------------------

    def wait_until_done(
        self,
        sid: int,
        timeout: float = 5.0,
        poll_hz: float = 50.0,
        settle_ms: float = 20.0,
    ) -> bool:
        """単一サーボの動作完了を待つ。

        位置モード（モード0）以外では「完了」概念がないため、即時 True を
        返す。コマンド送信直後の "まだ動き出し前で Moving=0" 誤判定を
        避けるため、ポーリング前に settle_ms だけ待ってから判定を始める。

        Args:
            sid: サーボID。
            timeout: タイムアウト [秒]。
            poll_hz: Moving レジスタのポーリング周波数 [Hz]。
            settle_ms: ポーリング開始前の初期待ち [ms]。

        Returns:
            停止確認できれば True、タイムアウト時 False。

        Raises:
            ValueError: ID未登録。
            NotConnectedError: 未接続。
        """
        self._require_connected()
        self._require_id(sid)

        # 位置モード以外では待っても意味がないので即時完了扱い
        if self._mode_cache.get(sid) != _MODE_POSITION:
            return True

        # 動き出しラグ吸収
        time.sleep(settle_ms / 1000.0)

        deadline = time.monotonic() + timeout
        interval = 1.0 / poll_hz
        while time.monotonic() < deadline:
            moving, comm, err = self._packet.ReadMoving(sid)
            self._check_result(sid, "Moving読取", comm, err)
            if comm == COMM_SUCCESS and moving == 0:
                return True
            time.sleep(interval)
        return False

    def wait_all_until_done(
        self,
        sids: Optional[list[int]] = None,
        timeout: float = 10.0,
        poll_hz: float = 50.0,
        settle_ms: float = 20.0,
    ) -> bool:
        """複数サーボの動作完了をまとめて待つ。

        位置モード以外のIDは即時に「完了」とみなして除外する。
        sids 省略時は接続時に登録した全IDが対象。

        Args:
            sids: 対象IDリスト。None で全登録ID。
            timeout: 全体のタイムアウト [秒]。
            poll_hz: Moving レジスタのポーリング周波数 [Hz]。
            settle_ms: ポーリング開始前の初期待ち [ms]。

        Returns:
            全ての対象IDが時間内に停止すれば True、タイムアウト時 False。

        Raises:
            ValueError: 未登録IDを含む。
            NotConnectedError: 未接続。
        """
        self._require_connected()

        # --- 対象IDの決定とバリデーション ---
        targets = list(sids) if sids is not None else list(self.ids)
        for sid in targets:
            self._require_id(sid)

        # --- 位置モードのIDだけを残す ---
        remaining = {
            sid for sid in targets if self._mode_cache.get(sid) == _MODE_POSITION
        }
        if not remaining:
            return True

        # --- 動き出しラグ吸収 ---
        time.sleep(settle_ms / 1000.0)

        # --- ポーリングループ（1巡で残り全IDをチェック） ---
        deadline = time.monotonic() + timeout
        interval = 1.0 / poll_hz
        while time.monotonic() < deadline and remaining:
            for sid in list(remaining):
                moving, comm, err = self._packet.ReadMoving(sid)
                self._check_result(sid, "Moving読取", comm, err)
                if comm == COMM_SUCCESS and moving == 0:
                    remaining.discard(sid)
            if remaining:
                time.sleep(interval)

        return len(remaining) == 0
