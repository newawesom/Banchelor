import PX4MavCtrlV4 as PX4MavCtrl
import time

mav = PX4MavCtrl.PX4MavCtrler(1)

mav.InitMavLoop()
time.sleep(1)
mav.SendPosNED(0, 0, -5, 0)
time.sleep(1)
mav.initOffboard()
mav.SendPosNED(0, 0, -5, 0)
time.sleep(0.5)
mav.SendMavArm(True)
time.sleep(1800)

print("Send offboard stop")
mav.endOffboard()
time.sleep(1)

#Exit MAVLink data receiving mode
print("Send Mavlink stop")
mav.stopRun()
time.sleep(1)