import UE4CtrlAPI as UE4CtrlAPI
import ReqCopterSim
import VisionCaptureApi
import cv2
import time, sys

req = ReqCopterSim.ReqCopterSim()
ue = UE4CtrlAPI.UE4CtrlAPI()
vis = VisionCaptureApi.VisionCaptureApi()

CopterID = 1
timeInterval = 1.0/30.0
lastTime = time.time()

#req.sendReSimMapName(CopterID, 'Factory_drone')
#time.sleep(5)
#req.sendReSimXyzRPYaw(CopterID, [4.5, 11.2, -1.21], [0, 0, 0])
#time.sleep(1)
vis.jsonLoad()

is_suss = vis.sendReqToUE4()
if not is_suss:
    print('[ERROR]Can not send request to UE4!')
    sys.exit(0)
vis.startImgCap(True)
ue.sendUE4Cmd('r.setres 960x540w') # 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
ue.sendUE4Cmd('t.MaxFPS 30') # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(2)    

width = 960
height = 540

while True:
    lastTime = lastTime + timeInterval
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime) # sleep until the desired clock
    else:
        lastTime = time.time()

    if vis.hasData[0]:
        img1 = vis.Img[0]
        cv2.imshow("ImageDown", img1)
        cv2.waitKey(1)
    if vis.hasData[1]:
        img2 = vis.Img[1]
        cv2.imshow("ImageFront", img2)
        cv2.waitKey(1)