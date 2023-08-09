import omni.ext
import omni.ui as ui
from omni.isaac.core import World
from carb.events import IEvent
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_at_path
from omni.isaac.core.utils.stage import traverse_stage

from omni.isaac.core.objects import GroundPlane
from omni.timeline import TimelineEventType
import carb
import omni.timeline
import numpy as np
from .zed_camera import ZEDCamera


# from pyzed_streamer import PyZEDStreamer


def log_verbose(msg: str):
    carb.log_warn(f"{msg}")
    return

def log_warn(msg: str):
    carb.log_warn(f"{msg}")
    return


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class ZEDCameraExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        log_verbose("Startup")
        if World.instance():
            self._world = World.instance()
        else:
            self._world = World()
        self._world.add_timeline_callback("zed_camera_callback", self.on_timeline_event)
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Nucleus srever with /Isaac folder")
        self._build_ui()
        self._zed_cameras: list = []
        self._loaded_cameras: dict = dict()
        self._used_ports: list= []

    def on_timeline_event(self, event: IEvent):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            log_verbose("Play pressed")
            cameras = self.find_cameras()
            if len(cameras) <= 0:
                log_verbose("No ZED cameras were detected.")
            else:
                for prim in cameras:
                    self.add_zed_camera_from_prim(prim)
        if event.type == int(omni.timeline.TimelineEventType.PAUSE):
            log_verbose("Pause pressed")
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            log_verbose("Stop pressed")
        if event.type == int(omni.timeline.TimelineEventType.CURRENT_TIME_TICKED):
            # do stuff here apparently
            self.stream_images()

        return

    def _build_ui(self):
        log_verbose("Creating UI")
        self._window = ui.Window("ZED Camera extension", width=300, height=300)
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                ui.Label("Debug controls")
                with ui.VStack(spacing=5):
                    ui.Button("Find cameras", clicked_fn=self.find_cameras)
                    ui.Button("Print info", clicked_fn=self.print_info)
                    ui.Button("Clear cameras", clicked_fn=self.clear_cameras)
        return


    def clear_cameras(self):
        self._zed_cameras.clear()
        self._used_ports.clear()
        self._loaded_cameras.clear()
        return

    def create_camera(self):
        camera_path = "/World"
        prim_name = f"ZEDCamera_{len(self._zed_cameras) + 1}"
        camera_to_add = ZEDCamera(
            cam_id=len(self._zed_cameras) + 1,
            prim_name=prim_name,
            camera_path=camera_path,
            position=[0.0, 0.0, 0.45],
            # orientation=[0.809, 0.0, 0.5878, 0.0]
        )
        self._zed_cameras.append(camera_to_add)
        return

    def is_zed_x_prim(self, prim):
        attributeCheck = prim.HasAttribute("StreamingPort") and prim.HasAttribute("DisableStreaming") and prim.HasAttribute("CameraName")
        if not attributeCheck:
            return False
        if not is_prim_path_valid(prim.GetPath().pathString + "/Imu_Sensor"):
            return False
        if not is_prim_path_valid(prim.GetPath().pathString + "/CameraLeft"):
            return False
        if not is_prim_path_valid(prim.GetPath().pathString + "/CameraRight"):
            return False
        return True

    def _get_new_streaming_port(self):
        if len(self._used_ports) == 0:
            return 30000
        else:
            self._used_ports.sort()
            return self._used_ports[len(self._used_ports) -1] +2

    def add_zed_camera_from_prim(self, prim):
        camera_name = prim.GetAttribute("CameraName").Get()
        disabled = prim.GetAttribute("DisableStreaming").Get()
        if disabled and (disabled == True):
            log_verbose(f"Camera {camera_name} disabled, skipping it")
            return
        log_verbose(f"Adding camera {camera_name}")
        cam_id = len(self._zed_cameras)
        if camera_name in self._loaded_cameras.keys():
            if self._loaded_cameras[camera_name] == prim.GetPath().pathString:
                log_verbose(f"Alread loaded {camera_name}")
                return
            else:
                new_name = camera_name + str(cam_id)
                # prim.SetAttribute(new_name)
                prim.GetAttribute("CameraName").Set(new_name)
                log_verbose(f"{camera_name} already loaded, its name will change to {new_name}.")
                camera_name = new_name
        streaming_port = prim.GetAttribute("StreamingPort").Get()
        if streaming_port:
            streaming_port = streaming_port.real
            if streaming_port in self._used_ports:
                streaming_port = self._get_new_streaming_port()
                log_verbose(f"Port used, defaulting to {streaming_port}")
        else:
            streaming_port = self._get_new_streaming_port()
            log_verbose(f"No streaming port set, defaulting to {streaming_port}")
        
        log_verbose(f"Creating camera <{camera_name}>")
        log_verbose(f"Streaming port <{streaming_port}>")
        log_verbose(f"Prim path <{prim.GetPath().pathString}>")
        log_verbose(f"Prim name <{prim.GetPath().name}>")
        to_add = ZEDCamera(
            cam_id=cam_id,
            ZED_X_prim_path=prim.GetPath().pathString,
            prim_name=prim.GetPath().name,
            streaming_port=streaming_port,
            frequency=10
        )
        self._zed_cameras.append(to_add)
        self._used_ports.append(streaming_port)
        self._loaded_cameras[camera_name] = prim.GetPath().pathString
        return

    def find_cameras(self):
        prims = []
        for prim in traverse_stage():
            if self.is_zed_x_prim(prim):
                log_verbose("Found ZED-X prim !")
                log_verbose(prim)
                prims.append(prim)
        return prims

    def print_info(self):
        _ = [log_verbose(cam.get_camera_parameters()) for cam in self._zed_cameras]
        return

    def stream_images(self):
        if len(self._zed_cameras) > 0:
            for cam in self._zed_cameras:
                if cam.is_new_data_available():
                    res = cam.stream_data()

        return

    def on_shutdown(self):
        if len(self._zed_cameras) > 0:
            for cam in self._zed_cameras:
                cam.dispose()
        if self._world:
            self._world.clear_timeline_callbacks()
            self._world.clear_render_callbacks()
            self._world.clear_all_callbacks()
            # self._world.reset()
        log_verbose("Shutdown")
