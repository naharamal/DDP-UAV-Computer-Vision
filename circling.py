import airsim
from scipy.spatial.transform import Rotation as ScipyRotation
import numpy as np

class Circling:
    def __init__(self):
        self.circling_client = airsim.MultirotorClient()
        self.circling_client.confirmConnection()
        self.circling_client.enableApiControl(True)
        self.circling_client.takeoffAsync()

        self.side_velocity  = 5

    def velocity_comand(self):
        drone_orientation = ScipyRotation.from_quat(self.circling_client.simGetVehiclePose().orientation.to_numpy_array())
        yaw = drone_orientation.as_euler('zyx')[0]
        forward_direction = np.array([np.cos(yaw), np.sin(yaw), 0])
        left_direction = np.array([np.cos(yaw - np.deg2rad(90)), np.sin(yaw - np.deg2rad(90)), 0])

        desired_velocity =

        self.control_client.moveByVelocityAsync(desired_vel[0], desired_vel[1], desired_vel[2], self.duration,
        drivetrain=airsim.DrivetrainType.ForwardOnly,yaw_mode=airsim.YawMode(True, yaw_rate))