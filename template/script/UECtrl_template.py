import UE4CtrlAPI as UE4CtrlAPI
import ReqCopterSim
import time

req = ReqCopterSim.ReqCopterSim()
ue = UE4CtrlAPI.UE4CtrlAPI()

CopterID = 1

req.sendReSimMapName(CopterID, 'Factory_drone')
time.sleep(5)
req.sendReSimXyzRPYaw(CopterID, [4.5, 11.2, -1.21], [0, 0, 0])

#ue.sendUE4Cmd(b'RflyChangeMapbyName Factory_drone')


