# STSController の使い方

`sts_controller.py` は、Feetech STS シリーズ（例: STS3215）のシリアルバスサーボを Python から扱いやすくするためのラッパーモジュールです。公式 SDK (`scservo_sdk`) の低レベルなレジスタ操作を隠し、角度指定、複数軸の同期移動、連続回転、PWM、トルク ON/OFF、温度・電圧・負荷などの状態取得をまとめて扱えます。

主な特徴は次の通りです。

- `with STSController(...) as ctrl:` で接続から切断までを自動管理できます。
- 位置制御、速度制御、PWM制御のモード切替を必要なタイミングで自動実行します。
- 速度・加速度・PWMを `0.0` から `1.0`、または `-1.0` から `1.0` の比率で指定できます。
- `angle_limits` によるソフトリミットで、危険な角度指令を防げます。
- `wait_until_done()` / `wait_all_until_done()` で位置移動の完了待ちができます。

## インストール

Feetech SDK が必要です。

```powershell
pip install ftservo-python-sdk
```

または、`scservo_sdk` が import できる状態にしてください。

## すぐ動かせるサンプル

次のコードは、1台または複数台のサーボで基本的な操作を一通り試すサンプルです。`PORT` と `SERVO_IDS` は自分の環境に合わせて変更してください。

```python
import time

from sts_controller import (
    CommError,
    STSController,
    SoftLimitError,
)


# Windows では "COM3" や "COM4"、Linux/macOS では "/dev/ttyUSB0" などを指定します。
PORT = "COM3"

# 接続するサーボIDを指定します。1台だけなら [1]、複数台なら [1, 2] のようにします。
SERVO_IDS = [1]

# 角度指定で動かす範囲をソフトウェア側で制限します。
# ここでは ID 1 のサーボを 20〜340度の範囲だけ動かせるようにしています。
ANGLE_LIMITS = {
    1: (20.0, 340.0),
}


try:
    # with 文を使うと、開始時に自動で connect()、終了時に自動で disconnect() されます。
    # disconnect() 時には全サーボのトルクがオフになります。
    with STSController(
        port=PORT,
        ids=SERVO_IDS,
        baudrate=1_000_000,
        angle_limits=ANGLE_LIMITS,
    ) as ctrl:
        sid = 1

        # ping で通信できるか確認します。
        print("ping:", ctrl.ping(sid))

        # 現在の状態を読み取ります。
        print("mode:", ctrl.get_mode(sid))              # 0:位置 / 1:速度 / 2:PWM / 3:ステップ
        print("position:", ctrl.get_position(sid), "deg")
        print("speed:", ctrl.get_speed(sid))            # 符号付きの速度生値
        print("load:", ctrl.get_load(sid))              # 符号付きの負荷生値
        print("voltage:", ctrl.get_voltage(sid), "V")
        print("temperature:", ctrl.get_temperature(sid), "C")

        # トルクをオンにして、位置制御モードで 180度へ動かします。
        ctrl.enable_torque(sid)
        ctrl.move_to_angle(
            sid,
            angle_deg=180.0,
            speed_ratio=0.3,  # 0.0〜1.0。大きいほど速い
            acc_ratio=0.3,    # 0.0〜1.0。大きいほど加速が強い
        )

        # 指令した位置に到達するまで待ちます。
        if ctrl.wait_until_done(sid, timeout=5.0):
            print("arrived:", ctrl.get_position(sid), "deg")
        else:
            print("move timed out")

        # 複数台を同時に角度指定する場合は move_to_angle_sync() を使います。
        # SERVO_IDS に ID 2 も含めた場合だけ実行します。
        if 2 in SERVO_IDS:
            ctrl.move_to_angle_sync(
                {
                    1: 120.0,
                    2: 240.0,
                },
                speed_ratio=0.4,
                acc_ratio=0.4,
            )
            ctrl.wait_all_until_done(timeout=5.0)

        # 連続回転モードで回します。
        # speed_ratio は -1.0〜1.0 で、符号が回転方向、絶対値が速さです。
        ctrl.move_with_speed(sid, speed_ratio=0.2, acc_ratio=0.3)
        time.sleep(2.0)
        ctrl.move_with_speed(sid, speed_ratio=0.0)  # 停止

        # PWM 開ループモードで動かします。
        # duty_ratio は -1.0〜1.0 で、符号が方向、絶対値が出力の強さです。
        ctrl.move_with_pwm(sid, duty_ratio=0.3)
        time.sleep(1.0)
        ctrl.move_with_pwm(sid, duty_ratio=0.0)  # 停止

        # 手で動かしたい場合や終了前に明示的に脱力したい場合はトルクをオフにします。
        ctrl.disable_torque(sid)

except SoftLimitError as exc:
    # angle_limits の範囲外へ動かそうとした場合に発生します。
    print("soft limit error:", exc)
except CommError as exc:
    # ポート、配線、電源、ID、ボーレートなどの通信問題で発生します。
    print("communication error:", exc)
```

## 基本的な考え方

`STSController` は、最初にポート名とサーボIDを指定して作成します。`with` 文で使うのが推奨です。

```python
from sts_controller import STSController

with STSController(port="COM3", ids=[1]) as ctrl:
    ctrl.move_to_angle(1, 180.0)
    ctrl.wait_until_done(1)
```

`with` 文を使わない場合は、必ず `connect()` と `disconnect()` を自分で呼びます。

```python
ctrl = STSController(port="COM3", ids=[1])
ctrl.connect()
try:
    ctrl.move_to_angle(1, 180.0)
finally:
    ctrl.disconnect()
```

## API リファレンス

### 例外

#### `FeetechError`

このモジュールで定義されている例外の基底クラスです。通信エラーやソフトリミットなど、モジュール由来の例外をまとめて捕捉したい場合に使います。

#### `CommError`

通信エラーを表します。ポートを開けない、ボーレートを設定できない、サーボが応答しない、読み書きに失敗した、サーボ側エラーが返った、などの場合に発生します。

#### `SoftLimitError`

`angle_limits` で設定した範囲外に `move_to_angle()` または `move_to_angle_sync()` で移動しようとした場合に発生します。

#### `NotConnectedError`

`connect()` する前に通信が必要なメソッドを呼んだ場合に発生します。通常は `with STSController(...) as ctrl:` の形で使えば避けられます。

### `STSController`

#### `STSController(port, ids, steps_per_rev=4095, max_speed=32767, max_acc=255, baudrate=1_000_000, angle_limits=None)`

コントローラを作成します。この時点ではまだ接続しません。

- `port`: シリアルポート名です。例: `"COM3"`, `"COM4"`, `"/dev/ttyUSB0"`。
- `ids`: 管理対象のサーボID一覧です。例: `[1]`, `[1, 2, 3]`。
- `steps_per_rev`: 1回転あたりのステップ数です。STS3215 の標準設定では `4095` です。
- `max_speed`: `speed_ratio=1.0` のときに使う速度の生値です。
- `max_acc`: `acc_ratio=1.0` のときに使う加速度の生値です。
- `baudrate`: 通信ボーレートです。標準では `1_000_000` です。
- `angle_limits`: IDごとの角度制限です。例: `{1: (20.0, 340.0)}`。

`ids` が空、または数値パラメータが不正な場合は `ValueError` が発生します。

### 接続・切断

#### `connect()`

シリアルポートを開き、ボーレートを設定し、登録された全IDに `ping` して存在確認します。さらに実機の現在モードを読み取り、内部キャッシュを初期化します。

通常は直接呼ばず、`with STSController(...) as ctrl:` を使うのがおすすめです。失敗時は `CommError` が発生します。

#### `disconnect()`

全サーボのトルクをオフにして、シリアルポートを閉じます。重複して呼んでも安全です。個別サーボのトルクオフに失敗しても、できる限りポートを閉じます。

#### `__enter__()` / `__exit__()`

`with` 文用のメソッドです。`__enter__()` で `connect()`、`__exit__()` で `disconnect()` が呼ばれます。

### 位置制御

#### `move_to_angle(sid, angle_deg, speed_ratio=0.5, acc_ratio=0.5)`

指定したサーボを指定角度へ動かします。内部で位置制御モード（モード0）へ切り替えます。

- `sid`: サーボIDです。
- `angle_deg`: 目標角度です。`0.0` から `360.0` の範囲で指定します。
- `speed_ratio`: 速度比です。`0.0` から `1.0` の範囲で指定します。
- `acc_ratio`: 加速度比です。`0.0` から `1.0` の範囲で指定します。

`speed_ratio` と `acc_ratio` は範囲外を指定しても内部でクランプされます。SDK仕様上、速度や加速度の生値 `0` が意図せず最大動作になることを避けるため、実際には最低値 `1` に丸められます。

`angle_deg` が `0.0` から `360.0` の範囲外なら `ValueError`、`angle_limits` の範囲外なら `SoftLimitError` が発生します。

#### `move_to_angle_sync(targets, speed_ratio=0.5, acc_ratio=0.5)`

複数サーボへ角度指令を一括送信します。個別に `move_to_angle()` を繰り返すより通信パケット数を減らせます。

- `targets`: `{id: angle_deg}` の辞書です。例: `{1: 120.0, 2: 240.0}`。
- `speed_ratio`: 全軸共通の速度比です。
- `acc_ratio`: 全軸共通の加速度比です。

全IDを位置制御モードに揃えてから `GroupSyncWrite` で送信します。`targets` が空の場合は何もしません。

### 連続回転・PWM制御

#### `move_with_speed(sid, speed_ratio, acc_ratio=0.5)`

連続回転モード（モード1）で速度指令を送ります。

- `sid`: サーボIDです。
- `speed_ratio`: 速度比です。`-1.0` から `1.0` の範囲で指定します。符号が回転方向、絶対値が速さです。`0.0` で停止します。
- `acc_ratio`: 加速度比です。`0.0` から `1.0` の範囲で指定します。

一定時間だけ回したい場合は、`move_with_speed()` の後に `time.sleep()` し、最後に `speed_ratio=0.0` を送って停止します。

#### `move_with_pwm(sid, duty_ratio)`

PWM 開ループモード（モード2）でデューティ指令を送ります。

- `sid`: サーボIDです。
- `duty_ratio`: PWMデューティ比です。`-1.0` から `1.0` の範囲で指定します。符号が方向、絶対値が出力の強さです。`0.0` で停止します。

内部では `-1000` から `1000` の生値に変換し、STS の方向ビット形式にエンコードして書き込みます。位置フィードバックを使わない開ループ制御なので、短時間の確認や特殊用途向けです。

### トルク制御

#### `enable_torque(sid)`

指定サーボのトルクをオンにします。位置保持や移動指令の前に明示的にオンにしたい場合に使います。

#### `disable_torque(sid)`

指定サーボのトルクをオフにします。手でサーボを動かしたい場合や、終了時に脱力したい場合に使います。`disconnect()` でも全サーボに対して自動的に呼ばれます。

### 状態取得

#### `ping(sid)`

指定サーボが応答するか確認します。応答があれば `True`、なければ `False` を返します。

#### `get_position(sid)`

現在位置を角度 `[deg]` で返します。連続回転モード中でも呼び出せますが、その場合は1回転内にラップされた角度として返ります。

#### `get_speed(sid)`

現在速度を符号付きの生値で返します。正負で回転方向、絶対値で速さを表します。

#### `get_load(sid)`

現在負荷を符号付きの生値で返します。おおよそ `±1000` が定格トルク 100% に相当する目安です。衝突検知や過負荷監視に使えます。

#### `get_temperature(sid)`

サーボ内部温度を摂氏 `[℃]` で返します。長時間運転では定期的な監視をおすすめします。

#### `get_voltage(sid)`

電源電圧 `[V]` を返します。生値は `電圧 × 10` の形式で読み取られ、メソッド内で `V` に変換されます。

#### `get_mode(sid)`

実機から現在の動作モードを読み取り、内部キャッシュも更新します。

返り値は次の通りです。

- `0`: 位置制御
- `1`: 連続回転・速度制御
- `2`: PWM 開ループ
- `3`: ステップ

### 完了待ち

#### `wait_until_done(sid, timeout=5.0, poll_hz=50.0, settle_ms=20.0)`

単一サーボの位置移動が完了するまで待ちます。停止確認できれば `True`、タイムアウトした場合は `False` を返します。

- `timeout`: タイムアウト秒数です。
- `poll_hz`: Moving レジスタを確認する頻度です。
- `settle_ms`: 指令直後の動き出しラグを避けるため、ポーリング開始前に待つ時間です。

位置制御モード以外では「到達完了」の概念がないため、即座に `True` を返します。

#### `wait_all_until_done(sids=None, timeout=10.0, poll_hz=50.0, settle_ms=20.0)`

複数サーボの位置移動完了をまとめて待ちます。全て停止確認できれば `True`、タイムアウトした場合は `False` を返します。

- `sids`: 対象IDのリストです。`None` の場合は、コンストラクタで登録した全IDが対象です。
- `timeout`: 全体のタイムアウト秒数です。
- `poll_hz`: Moving レジスタを確認する頻度です。
- `settle_ms`: ポーリング開始前に待つ時間です。

位置制御モード以外のIDは完了待ちの対象から除外されます。

## 注意点

- このクラスはスレッドセーフではありません。複数スレッドから同時に操作する場合は、呼び出し側で排他制御してください。
- モード切替はサーボの EEPROM 領域を書き換えます。このモジュールは前回モードをキャッシュし、必要な場合だけモードを書き換えることで EEPROM 書き込みを抑えています。
- `move_with_speed()` と `move_with_pwm()` は、停止指令を送るまで回り続けます。試すときは短い `sleep` の後に必ず `0.0` を送って停止してください。
- `disconnect()` すると全サーボのトルクがオフになります。位置保持したまま終了したい用途では、この挙動に注意してください。
