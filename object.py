import airsim
import time
import pprint

class object:
    def __init__(self):
        client = airsim.VehicleClient()
        client.confirmConnection()


        pose1 = client.simGetObjectPose("OrangeBall")
        for i in range(1000):
            pose1.position = pose1.position + airsim.Vector3r(-0.1, 0, 0)
            success = client.simSetObjectPose("OrangeBall", pose1, True)

# object()