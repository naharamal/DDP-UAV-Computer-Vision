import airsim
from pynput import keyboard
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
import time
locations = []
import pickle as pkl
import threading
import cv2



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
                "area_error" : 0

                }
        self.control_params = {
                "k" :0.001
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

        self.t1 = threading.Thread(target= self.controller)
        self.t2 = threading.Thread(target= self.image_processing)

        self.control_state = False

        self.trackers = cv2.MultiTracker_create()

    def controller(self):
        while(self.control_state):
            desired_vel = [0.0,0.0,0.0]
            if(self.error["y_error"]>0):
                desired_vel[2] = -4*self.error["y_error"]/72
            if(self.error["y_error"]<0):
                desired_vel[2] = -1*4*self.error["y_error"]/72
            if(self.error["x_error"]>0):
                desired_vel[1] = -8*self.error["x_error"]/128
            if(self.error["x_error"]<0):
                desired_vel[1] = -8*self.error["x_error"]/128


            if(self.error["area_error"]>0):
                desired_vel[0] = 2*self.error["area_error"]/(72*128*2)
            # if(self.error["x_error"]<0):
            #     desired_vel[0] = -8*self.error["x_error"]/128

            self.control_client.moveByVelocityAsync(desired_vel[0], desired_vel[1], desired_vel[2], self.duration,
            drivetrain=airsim.DrivetrainType.ForwardOnly,yaw_mode=airsim.YawMode(True, 0.0))
            time.sleep(0.01)






    def fly_by_keyboard(self):
        with keyboard.Listener(on_press=self._on_press, on_release=self._on_release) as keyboard_listener:
            keyboard_listener.wait()
            print("Ready, you can control the drone by keyboard now.")
            while keyboard_listener.running:
                self._handle_commands()
                time.sleep(self.duration / 2.0)
            keyboard_listener.join()
        print("Manual control mode was successfully deactivated.")

    def move(self, velocity, yaw_rate):
        self._client.moveByVelocityAsync(velocity[0].item(), velocity[1].item(), velocity[2].item(), self.duration,
                                         drivetrain=airsim.DrivetrainType.ForwardOnly,
                                         yaw_mode=airsim.YawMode(True, yaw_rate))

    def image_processing(self):
        while(True):
            rawImage = self.img_client.simGetImage("0", airsim.ImageType.Scene)
            if (rawImage == None):
                print("Camera is not returning image, please check airsim for error messages")
            else:
                frame = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
                (success, boxes) = self.trackers.update(frame)

            for box in boxes:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.error["y_error"] = 72 - y - h/2
                self.error["x_error"] = 128 - x - w/2
                self.error["area_error"] = 72*128 - w*h
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("b"):
                box = cv2.selectROI("Frame", frame, fromCenter=False,
                    showCrosshair=True)
                self.tracker = cv2.TrackerMedianFlow_create()
                self.trackers.add(self.tracker, frame, box)
            elif key == ord("q"):
                cv2.destroyAllWindows()
                break

    def _on_press(self, key):
        if key in self._key_command_mapping.keys():
            self._active_commands[self._key_command_mapping[key]] = True
        if key == keyboard.KeyCode.from_char("c"):
            self.control_state = True
            print("start controller")
            self.t1.start()
            self.t2.start()
            print("processes started")

        elif key == keyboard.KeyCode.from_char("x"):
            print("stop controller")
            self.control_state = False

        elif key == keyboard.Key.esc:
            self.control_state = False
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