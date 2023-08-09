import carb
from omni.isaac.sensor import Camera, _sensor, IMUSensor
from omni.isaac.core.prims import XFormPrim, RigidPrim
import numpy as np
from omni.isaac.core.utils.stage import add_reference_to_stage
import os
from sl.sensor.camera.pyzed_streamer import PyZEDStreamer
import copy
from time import time 

BASELINE = 0.119886
STREAM_PORT = 30000
SERIAL_NUMBER = 31624334
ZED_X_USD_PATH = os.path.abspath(os.path.join(os.path.abspath(__file__), "..", "..", "data", "zed_x.usd"))


class ZEDCamera:
    def __init__(
        self,
        cam_id: int,
        ZED_X_prim_path="/World",
        prim_name="ZEDCamera",
        frequency: int = 10,
        position=None,
        orientation=None,
        streaming_port = STREAM_PORT,
    ) -> None:
        self._zed_x_prim_path = ZED_X_prim_path
        self._name = prim_name
        self._cameraL: Camera = None
        self._cameraR: Camera = None
        self._imu_sensor_interface = _sensor.acquire_imu_sensor_interface()
        self._imuSensor: IMUSensor = None
        self._zed_camera_prim: XFormPrim = None
        self._frequency = frequency
        self._position = position
        self._id = cam_id
        if not self._position:
            self._position = [0.0, 0.0, 0.0]
        self._orientation = orientation
        self._last_capture_frame_time = 0
        self._last_timestamp = 0.0
        self._initial_timestamp = 0.0
        self._streaming_port = streaming_port
        self._streamer: PyZEDStreamer = PyZEDStreamer()
        self._streamer.init(
            image_width=1280, image_height=720, port=self._streaming_port, serial_number=SERIAL_NUMBER)
        carb.log_warn(f"Creating camera {SERIAL_NUMBER} - {cam_id}")
        self._create_camera()


    @property
    def get_streaming_port(self):
        return self._streaming_port

    @property
    def get_path(self):
        return self._zed_x_prim_path
    
    @property
    def get_id(self):
        return self._id

    def _log(self, msg):
        carb.log_warn(msg)

    def _set_camera_properties(self, camera: Camera, isLeft=True):
        role = "left" if isLeft else "right"
        msg = "\n"
        msg = msg + f"Stereo role: {role}\n"
        msg = msg + f"frequency role: {self._frequency} Hz"
        camera.set_stereo_role(role)
        camera.set_resolution([1280, 720])
        camera.set_frequency(self._frequency)  # --fps
        camera.set_focal_length(2.694675)  # -- float
        camera.set_focus_distance(33.6)  # -- float
        # camera.set_fisheye_polynomial_properties() # A1 A2 A3 A4 A5
        # camera.set_projection_mode() # --perspective or orthographic
        # camera.set_projection_type() # -- pinhole | fisheyeOrthographic | fisheyeEquidistant | fisheyePolynomial | fisheyeSpherical
        # camera.set_shutter_properties(delay_open, delay_close)
        camera.set_horizontal_aperture(4.92741)  # --float
        camera.set_vertical_aperture(2.77167)  # --float
        camera.set_clipping_range(near_distance=0.01, far_distance=100000)
        camera.set_visibility(False)
        self._log(msg)
        return

    def calculate_intrinsics(self):
        height = 720
        width = 1280
        focal_length = self._cameraL.get_focal_length()
        aperture_vertical = self._cameraL.get_vertical_aperture()
        aperture_horizontal = self._cameraL.get_horizontal_aperture()
        fx = height * focal_length / aperture_vertical
        fy = width * focal_length / aperture_horizontal
        center_x = height * 0.5
        center_y = width * 0.5
        return fx, fy, center_x, center_y

    def get_camera_parameters(self):
        msg = "Camera parameters \n"
        msg = msg + "\nStereo role " + f"{self._cameraL.get_stereo_role()}"
        msg = msg + "\nResolution " + f"{self._cameraL.get_resolution()}"
        msg = msg + "\nFocal length " + f"{self._cameraL.get_focal_length()}"  # -- float
        msg = msg + "\nFocus distance " + f"{self._cameraL.get_focus_distance()}"  # -- float
        msg = msg + "\nHorizontal aperture " + f"{self._cameraL.get_horizontal_aperture()}"  # --float
        msg = msg + "\nVertical aperture " + f"{self._cameraL.get_vertical_aperture()}"  # --float
        msg = msg + "\nLens aperture" + f"{self._cameraL.get_lens_aperture()}"  # --float
        msg = msg + "\nFOV Horizontal " + f"{self._cameraL.get_horizontal_fov()}"  # --float
        msg = msg + "\nFOV Vertical " + f"{self._cameraL.get_vertical_fov()}"  # --float
        msg = msg + "\nIntrinsics:\n" + f"{self._cameraL.get_intrinsics_matrix()}"   # --float
        calculated_intrinsics = self.calculate_intrinsics()
        msg = msg + "\nCalculated Intrinsics:\n" + f"fx {calculated_intrinsics[0]}\nfy {calculated_intrinsics[1]}\n"   
        msg = msg + f"cx {calculated_intrinsics[2]}\ncy {calculated_intrinsics[3]}\n"   
        return msg

    def reset_timestamp(self):
        self._last_capture_frame_time = 0
        self._last_timestamp = 0.0
        self._initial_timestamp = 0.0
        return

    def _create_camera(self):
        # zed_camera_prim_path = self._camera_path
        # rigid_prim_path =   self._camera_path +  "/base_link"
        # zed_x_prim_path = self._camera_path +  "/base_link/ZED_X"
        # # camera_link_prim_path = zed_camera_prim_path + "/" + "camera_mount"
        # # camera_link_prim_name = "camera_mount"

        # self._log(f"Creating ZED camera at {self._camera_path}")
        # zed_camera_prim = XFormPrim(
        #     prim_path=zed_camera_prim_path, name=self._name, position=self._position, orientation=self._orientation
        # )
        

        # self._log(f"Importing zed_x prim at {zed_camera_prim_path}")
        # add_reference_to_stage(
        #     usd_path=ZED_X_USD_PATH, prim_path=zed_camera_prim_path
        # )  # Dont change this, it makes sense in the long run
        # zed_x_prim = RigidPrim(
        #     prim_path=rigid_prim_path, name=self._name, position=self._position, orientation=self._orientation
        # )
        # self._log(f"Initializing zed_x RigidBody prim")
        # zed_camera_prim.initialize()

        # self._log(f"Creating imu sensor at: {self._camera_path}")
        self._imuSensor = IMUSensor(
            prim_path=self._zed_x_prim_path + "/" + "Imu_Sensor",
            name="IMUSensor",
            frequency=self._frequency,
        )

        camera_orientation = [1.0, 0.0, 0.0, 0.0]
        # camera_orientation = [0.5, -0.5, -0.5, 0.5]
        positionL = [0 for x in self._position]
        positionR = [0 for x in self._position]
        positionL[0] = positionL[0] - BASELINE / 2
        positionR[0] = positionR[0] + BASELINE / 2
        positionL[2] = positionL[2] + 0.025
        positionR[2] = positionR[2] + 0.025
        prim_left = "CameraLeft"
        prim_right = "CameraRight"
        path_cameraL = self._zed_x_prim_path + "/" + prim_left
        path_cameraR = self._zed_x_prim_path + "/" + prim_right

        def log_camera_info(path, name, position, orientation):
            msg = f"Creating {name} at {path}, position: {position}, orientation: {orientation}."
            self._log(msg)
            return

        # self._cameraL = Camera(path_cameraL, prim_left, translation=positionL, orientation=camera_orientation)
        self._cameraL = Camera(path_cameraL, prim_left)
        log_camera_info(path_cameraL, prim_left, positionL, camera_orientation)
        self._set_camera_properties(self._cameraL, isLeft=True)
        self._cameraL.initialize()
        # self._cameraR = Camera(path_cameraR, prim_right, translation=positionR, orientation=camera_orientation)
        self._cameraR = Camera(path_cameraR, prim_right)
        log_camera_info(path_cameraR, prim_right, positionR, camera_orientation)
        self._set_camera_properties(self._cameraR, isLeft=False)
        self._cameraR.initialize()
        return

    def grab_image(self, is4chanel=False) -> np.ndarray:
        if self._cameraL and self._cameraR:
            if is4chanel:
                data_left = self._cameraL.get_current_frame()["rgba"]
                data_right = self._cameraR.get_current_frame()["rgba"]
            else:
                data_left = self._cameraL.get_current_frame()["rgba"][:, :, :3]
                data_right = self._cameraR.get_current_frame()["rgba"][:, :, :3]
            self._last_capture_frame_time = self._cameraL.get_current_frame()["rendering_time"]
            return data_left, data_right
        else:
            self._log("Tried to get data when no camera is created.")
            return None

    def stream_data(self) -> int:
        if self._cameraL:
            left: np.ndarray = None
            right: np.ndarray = None
            res = self.grab_image()
            lin_acc: list = []
            orientation: list = []
            if res:
                if self._imuSensor:
                    lin_acc, _, orientation = self.read_imu_data()
                    temp_orientation = self._imuSensor.get_world_pose()[1]
                    swp = copy.deepcopy(temp_orientation)
                    orientation = [swp[0], -swp[1], -swp[3], -swp[2]]
                ts = int(self.get_latest_timestamp())
                left, right = res[0], res[1]
                self._log(
                    f"ZED Cam {self._id}-{ts} Streaming images ({left.shape} - {right.shape}) acceleration {lin_acc} and orientation {orientation}"
                )
                res = self._streamer.stream(
                    left.tobytes(),
                    right.tobytes(),
                    ts,
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    orientation[3],
                    lin_acc[0],
                    lin_acc[1],
                    lin_acc[2],
                )
                return res
        return -1

    def grap_pose(self):
        return self._cameraL.get_world_pose()

    def reset(self):
        self.reset_timestamp()
        return

    def is_new_data_available(self) -> bool:
        current_frame_timeL = self._cameraL.get_current_frame()["rendering_time"]
        current_frame_timeR = self._cameraR.get_current_frame()["rendering_time"]
        if current_frame_timeL > self._last_capture_frame_time:
            return True
        if current_frame_timeR > self._last_capture_frame_time:
            return True
        return False

    def read_imu_data(self):
        imu_sensor_reading = self._imu_sensor_interface.get_sensor_readings(self._imuSensor.prim_path)
        if imu_sensor_reading.shape[0]:
            lin_acc = [
                imu_sensor_reading["lin_acc_x"][0],
                imu_sensor_reading["lin_acc_y"][0],
                imu_sensor_reading["lin_acc_z"][0],
            ]
            ang_vel = [
                imu_sensor_reading["ang_vel_x"][0],
                imu_sensor_reading["ang_vel_y"][0],
                imu_sensor_reading["ang_vel_z"][0],
            ]
            orientation = [
                imu_sensor_reading["orientation"][0][3],
                imu_sensor_reading["orientation"][0][0],
                imu_sensor_reading["orientation"][0][1],
                imu_sensor_reading["orientation"][0][2],
            ]
            return lin_acc, ang_vel, orientation
        return None, None, None

    def get_latest_timestamp(self):
        if self._initial_timestamp <= 0.0:
            self._initial_timestamp = time() * 1000.0
        self._log(int(self._last_capture_frame_time * 1000))
        self._last_timestamp = self._initial_timestamp +  int(self._last_capture_frame_time * 1000)
        return self._last_timestamp

    def force_get_rgba(self):
        self._log("Rendering and Grabbing images")
        if self._cameraL and self._cameraR:
            self._log("Grabbing left image")
            data_left = self._cameraL.get_rgba()[:, :, :3]
            self._log("Grabbing right image")
            data_right = self._cameraL.get_rgba()[:, :, :3]
            return data_left, data_right
        else:
            self._log("Tried to get data when no camera is created.")
        return None

    def dispose(self):

        self._log(f"Disposing of ZEDCamera.")
        return
