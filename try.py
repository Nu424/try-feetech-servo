import time
from vassar_feetech_servo_sdk import ServoController

def wait_until_stopped(c: ServoController, motor_id: int, timeout: float = 10.0, poll_hz: int = 50, settle_ms: int = 20) -> bool:
    """
    指定IDのサーボが停止するまで待つ。
    settle_ms: 命令直後の "まだ動き出してないのにMoving=0" を避けるための初期待ち
    """
    time.sleep(settle_ms / 1000)  # 移動開始ラグの吸収
    deadline = time.monotonic() + timeout
    interval = 1.0 / poll_hz

    while time.monotonic() < deadline:
        moving, comm, err = c.packet_handler.ReadMoving(motor_id)
        if comm == 0 and err == 0 and moving == 0:
            return True
        time.sleep(interval)
    return False  # タイムアウト

with ServoController(servo_ids=[1], servo_type="sts", port="COM3") as c:
    for pos in [0, 1024, 2048, 3072, 4094, 0]:
        c.write_position({1: pos}, speed=600, acceleration=255)
        wait_until_stopped(c, 1)
    print("着いた:", c.read_position(1))
    time.sleep(1)

    # c.set_operating_mode(1, 2)

    # # PWMを流す。値の範囲は -1000〜+1000（BIT10が方向ビット）
    # SMS_STS_RUNNING_TIME   = 0x2C  # 44, PWMモード時はPWMデューティとして機能
    # def write_pwm(id_, pwm, ph):
    #     # pwmを符号付き2バイトとして書く。SDKに符号付き変換ヘルパが大体ある
    #     # 無ければ自前で：負なら 1<<10 (=1024) を立てて絶対値部分と合成
    #     if pwm < 0:
    #         raw = (1 << 10) | (abs(pwm) & 0x3FF)
    #     else:
    #         raw = pwm & 0x3FF
    #     ph.write2ByteTxRx(id_, SMS_STS_RUNNING_TIME, raw)

    # write_pwm(1, 1000, c.packet_handler)    # 50%デューティで正方向
    # time.sleep(2)
    # write_pwm(1, -1000, c.packet_handler)   # 80%デューティで逆方向
    # time.sleep(2)
    # write_pwm(1, 0, c.packet_handler)      # 停止

    
    c.set_operating_mode(1, 1)
    c.packet_handler.WriteSpec(1, 600, 255)
    time.sleep(3)
    c.packet_handler.WriteSpec(1, -600, 255)
    time.sleep(3)
    c.packet_handler.WriteSpec(1, 0, 50)
    time.sleep(1)
    # c.set_operating_mode(1, 0)