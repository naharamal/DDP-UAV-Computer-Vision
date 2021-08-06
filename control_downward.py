import airsim
from pynput import keyboard
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
import time
locations = []
import pickle as pkl
import threading
import cv2
import math
# from plot import plotting
import object
from PIL import Image as im

OPENCV_OBJECT_TRACKERS = {
	"csrt": cv2.TrackerCSRT_create,
	"kcf": cv2.TrackerKCF_create,
	"boosting": cv2.TrackerBoosting_create,
	"mil": cv2.TrackerMIL_create,
	"tld": cv2.TrackerTLD_create,
	"medianflow": cv2.TrackerMedianFlow_create,
	"mosse": cv2.TrackerMOSSE_create,
    "go_turn":cv2.TrackerGOTURN_create

}


error_list = []
class DroneController:

    def __init__(self):
        self.acceleration = 3.0
        self.max_speed = 6.0
        self.angular_velocity = 90.0
        self.duration = 0.4
        self.friction = 0.5

        self.desired_velocity = np.zeros(3, dtype=np.float32)

        self._key_command_mapping = {
            keyboard.Key.up: "forward",
            keyboard.Key.down: "backward",
            keyboard.Key.left: "turn left",
            keyboard.Key.right: "turn right",
            keyboard.KeyCode.from_char("w"): "up",
            keyboard.KeyCode.from_char("s"): "down",
            keyboard.KeyCode.from_char("a"): "left",
            keyboard.KeyCode.from_char("d"): "right",
        }

        self.error = {
                "y_error": 0,
                "x_error": 0,
                "area_error" : 0,

                "y_error_sum": 0,
                "x_error_sum": 0,
                "area_error_sum" : 0,

                "y_error_prev": 0,
                "x_error_prev": 0,
                "area_error_prev" : 0,

                "y_error_diff": 0,
                "x_error_diff": 0,
                "area_error_diff" : 0

                }
        self.control_params = {
                "kpy" : -6/72.0,
                "kpx" : -90.0/128.0,
                "kpa" : 8/(72*128*2),

                "kdy" : -0.01,
                "kdx" : -0.01,
                "kda" : -0.0,

                "kiy" : 0.0,
                "kix" : 0.0,
                "kia" : 0.0,
                }

        self._active_commands = {command: False for command in self._key_command_mapping.values()}
        self._client = airsim.MultirotorClient()
        self._client.confirmConnection()
        self._client.enableApiControl(True)
        self._client.takeoffAsync()

        self.control_client = airsim.MultirotorClient()
        self.control_client.confirmConnection()
        self.control_client.enableApiControl(True)
        self.control_client.takeoffAsync()


        self.img_client = airsim.MultirotorClient()
        self.img_client.confirmConnection()
        self.img_client.enableApiControl(True)
        self.img_client.armDisarm(True)
        self.img_client.takeoffAsync().join()

        self.cam_client = airsim.MultirotorClient()
        self.cam_client.confirmConnection()
        self.cam_client.enableApiControl(True)
        self.cam_client.armDisarm(True)
        self.cam_client.takeoffAsync().join()

        self.t1 = threading.Thread(target= self.controller)
        self.t2 = threading.Thread(target= self.image_processing)

        self.control_state = False

        self.thread_start = False

        self.circling = False

        self.side_velocity = 2

        self.trackers = cv2.MultiTracker_create()

        self.camera_angle = 0

        camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(-30), 0, 0)) #radians
        self.cam_client.simSetCameraPose("0", camera_pose)

        pose1 = self.cam_client.simGetObjectPose("OrangeBall")
        print(pose1)




    def camera_control(self):
        self.camera_angle += 1
        camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(self.camera_angle), 0, 0)) #radians
        self.cam_client.simSetCameraPose("3", camera_pose)

    def controller(self):
        while(self.thread_start):
            while(self.control_state):
                desired_vel = [0.0,0.0,0.0]
                yaw_rate = 0

                drone_orientation = ScipyRotation.from_quat(self.control_client.simGetVehiclePose().orientation.to_numpy_array())
                yaw = drone_orientation.as_euler('zyx')[0]
                forward_direction = np.array([np.cos(yaw), np.sin(yaw), 0])
                left_direction = np.array([np.cos(yaw - np.deg2rad(90)), np.sin(yaw - np.deg2rad(90)), 0])

                desired_vel[2] = self.control_params["kpy"]*self.error["y_error"] +\
                                self.control_params["kdy"]*self.error["y_error_diff"] +\
                                self.control_params["kiy"]*self.error["y_error_sum"]
                yaw_rate = self.control_params["kpx"]*self.error["x_error"] +\
                        self.control_params["kdx"]*self.error["x_error_diff"] +\
                        self.control_params["kix"]*self.error["x_error_sum"]
                desired_vel += self.control_params["kpa"]*self.error["area_error"]*forward_direction +\
                            self.control_params["kda"]*self.error["area_error_diff"]*forward_direction +\
                            self.control_params["kia"]*self.error["area_error_sum"]*forward_direction
                if(self.circling):
                    desired_vel += self.side_velocity*left_direction

                desired_vel = np.clip(desired_vel,-8,+8)

                self.control_client.moveByVelocityAsync(desired_vel[0], desired_vel[1], desired_vel[2], self.duration,
                drivetrain=airsim.DrivetrainType.ForwardOnly,yaw_mode=airsim.YawMode(True, yaw_rate))
                time.sleep(0.01)

    def fly_by_keyboard(self):
        with keyboard.Listener(on_press=self._on_press, on_release=self._on_release) as keyboard_listener:
            keyboard_listener.wait()
            print("Ready, you can control the drone by keyboard now.")
            while keyboard_listener.running:
                self._handle_commands()
                self._logData()
                time.sleep(self.duration / 2.0)
            keyboard_listener.join()
        print("Manual control mode was successfully deactivated.")

    def move(self, velocity, yaw_rate):
        self._client.moveByVelocityAsync(velocity[0].item(), velocity[1].item(), velocity[2].item(), self.duration,
                                         drivetrain=airsim.DrivetrainType.ForwardOnly,
                                         yaw_mode=airsim.YawMode(True, yaw_rate))

    def _logData(self):
        state = self._client.getMultirotorState()
        loc = [state.kinematics_estimated.position.x_val,state.kinematics_estimated.position.y_val,state.kinematics_estimated.position.z_val]
        locations.append(loc)

    def image_processing(self):
        while(True):
                rawImage = self.img_client.simGetImage("0", airsim.ImageType.Scene)
                if (rawImage == None):
                    print("Camera is not returning image, please check airsim for error messages")
                else:
                    frame = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_COLOR)
                    (success, boxes) = self.trackers.update(frame)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                detected_circles = cv2.HoughCircles(gray,
                   cv2.HOUGH_GRADIENT , 1, 20, param1 = 50,
                            param2 = 50, minRadius = 1, maxRadius = 70)

                for box in boxes:
                    (x, y, w, h) = [int(v) for v in box]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    self.error["y_error"] = 110 - y - h/2
                    self.error["x_error"] = 128 - x - w/2
                    self.error["area_error"] = 72*128/2 - w*h

                    error_list.append([self.error["y_error"],self.error["x_error"],self.error["area_error"]])

                    self.error["y_error_sum"] += self.error["y_error"]
                    self.error["x_error_sum"] += self.error["x_error"]
                    self.error["area_error_sum"] += self.error["area_error"]

                    self.error["y_error_diff"] += self.error["y_error"]-self.error["y_error_prev"]
                    self.error["x_error_diff"] += self.error["x_error"]-self.error["x_error_prev"]
                    self.error["area_error_diff"] += self.error["area_error"]-self.error["area_error_prev"]

                    self.error["y_error_prev"] = self.error["y_error"]
                    self.error["x_error_prev"] = self.error["x_error"]
                    self.error["area_error_prev"] = self.error["area_error"]

                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("b"):
                    box = cv2.selectROI("Frame", frame, fromCenter=False,
                        showCrosshair=True)
                    self.tracker = OPENCV_OBJECT_TRACKERS["csrt"]()
                    # self.tracker = cv2.TrackerKCF_create()
                    self.trackers = cv2.MultiTracker_create()
                    self.trackers.add(self.tracker, frame, box)
                elif key == ord("q"):
                    cv2.destroyAllWindows()
                    break

    def _on_press(self, key):
        if key in self._key_command_mapping.keys():
            self._active_commands[self._key_command_mapping[key]] = True
        if key == keyboard.KeyCode.from_char("c"):
            self.control_state = True
            if not self.thread_start:
                self.t2.start()
                print("image started")
        if key == keyboard.KeyCode.from_char("m"):
                self.thread_start = True
                self.t1.start()
                print("start controller")

        if key == keyboard.KeyCode.from_char("u"):
            self.circling = True

        if key == keyboard.KeyCode.from_char("p"):
            self.camera_control()

        elif key == keyboard.KeyCode.from_char("x"):
            print("stop controller")
            # self.control_state = not self.control_state
            object.object()

        elif key == keyboard.Key.esc:
            self.control_state = False
            self.thread_start = False
            return False

    def _on_release(self, key):
        if key in self._key_command_mapping.keys():
            self._active_commands[self._key_command_mapping[key]] = False


    def _handle_commands(self):
        drone_orientation = ScipyRotation.from_quat(self._client.simGetVehiclePose().orientation.to_numpy_array())
        yaw = drone_orientation.as_euler('zyx')[0]
        forward_direction = np.array([np.cos(yaw), np.sin(yaw), 0])
        left_direction = np.array([np.cos(yaw - np.deg2rad(90)), np.sin(yaw - np.deg2rad(90)), 0])

        if self._active_commands["forward"] or self._active_commands["backward"]:
            forward_increment = forward_direction * self.duration * self.acceleration
            if self._active_commands["forward"]:
                self.desired_velocity += forward_increment
            else:
                self.desired_velocity -= forward_increment
        else:
            forward_component = np.dot(self.desired_velocity, forward_direction) * forward_direction
            self.desired_velocity -= self.friction * forward_component

        if self._active_commands["up"] or self._active_commands["down"]:
            vertical_component = drone_orientation.apply(np.array([0.0, 0.0, -1.0]))
            vertical_component *= self.duration * self.acceleration
            if self._active_commands["up"]:
                self.desired_velocity += vertical_component
            else:
                self.desired_velocity -= vertical_component
        else:
            self.desired_velocity[2] *= self.friction

        if self._active_commands["left"] or self._active_commands["right"]:
            lateral_increment = left_direction * self.duration * self.acceleration
            if self._active_commands["left"]:
                self.desired_velocity += lateral_increment
            else:
                self.desired_velocity -= lateral_increment
        else:
            left_component = np.dot(self.desired_velocity, left_direction) * left_direction
            self.desired_velocity -= self.friction * left_component

        speed = np.linalg.norm(self.desired_velocity)
        if speed > self.max_speed:
            self.desired_velocity = self.desired_velocity / speed * self.max_speed

        yaw_rate = 0.0
        if self._active_commands["turn left"]:
            yaw_rate = -self.angular_velocity
        elif self._active_commands["turn right"]:
            yaw_rate = self.angular_velocity

        self.move(self.desired_velocity, yaw_rate)


if __name__ == "__main__":
    controller_1 = DroneController()
    controller_1.fly_by_keyboard()

    with open("error.pkl",'wb') as f:
        pkl.dump(error_list ,f)
    # plotting()

    with open("trajectory.pkl",'wb') as f:
        pkl.dump(locations ,f)

