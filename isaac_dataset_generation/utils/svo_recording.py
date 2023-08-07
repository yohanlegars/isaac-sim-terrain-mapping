import sys
import pyzed.sl as sl
from signal import signal, SIGINT
import threading
import yaml

# from TerrainMeshRecorder import parser


# stream = open(sys.argv[1])
# print(sys.argv[1])
# parser = yaml.load(stream)

#def record_svo(ip):

ip = "192.168.2.183"
    
init = sl.InitParameters()

init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.NONE



init.set_from_stream(ip)
cam = sl.Camera()

error = True
while error == True:
    print("Trying to open the camera")
    status = cam.open(init)

    if status != sl.ERROR_CODE.SUCCESS:
     print(f"Failed to open camera, error code: {repr(status)}")
    
    else:
          error = False

path_output = sys.argv[1]
recording_param = sl.RecordingParameters(path_output, sl.SVO_COMPRESSION_MODE.H264, transcode_streaming_input=True)
err = cam.enable_recording(recording_param)
if err != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    
            
runtime = sl.RuntimeParameters()
print("SVO is Recording, press q to stop.")
frames_recorded = 0

while True:
     if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                    frames_recorded += 1
                    print("Frame count: " + str(frames_recorded), end="\r")


    #return cam, runtime
    
           