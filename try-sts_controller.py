from sts_controller import STSController
import time

# 接続して基本確認
with STSController(port="COM3", ids=[1]) as ctrl:
    print("voltage:", ctrl.get_voltage(1), "V")
    print("temp:", ctrl.get_temperature(1), "C")
    print("mode:", ctrl.get_mode(1))

    # モード0: 180度へ
    ctrl.move_to_angle(1, 180.0, speed_ratio=0.3, acc_ratio=0.3)
    if ctrl.wait_until_done(1):
        print("着いた:", ctrl.get_position(1))

    # モード1: 連続回転
    ctrl.move_with_speed(1, speed_ratio=0.2, acc_ratio=0.3)
    time.sleep(2)
    ctrl.move_with_speed(1, 0)

    # モード2: PWM
    ctrl.move_with_pwm(1, duty_ratio=0.3)
    time.sleep(1)
    ctrl.move_with_pwm(1, 0)
