
import os
import sys
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import style
import json
import copy
import pyzed.sl as sl
import ogl_viewer.viewer as gl
import open3d as o3d

# Set this variable to 'True' for a mesh viewer
# or to 'False' for a point cloud viewer 

CREATE_MESH = True 

class Evaluation:

    '''
    Evaluation module

    Stores the evaluation curves and can save them to matplotlib figs
    '''
    def __init__(self, threshold_range, beta=0.25):
        self.threshold_range = threshold_range
        self.distance1 = None 
        self.distance2 = None 
        self.percisions = None
        self.recalls = None
        self.fscores = None
        self.beta = beta



    def load_meshes(self, gt_mesh, test_mesh):

        t = copy.deepcopy(test_mesh)
        s = copy.deepcopy(gt_mesh)

        self.distance1 = s.compute_point_cloud_distance(t)
        self.distance2 = t.compute_point_cloud_distance(s)


    def compute_prec_rec(self):
        self.precisions = []
        self.recalls = []

        for threshold in self.threshold_range:

            self.recalls.append(float(sum(d < threshold for d in self.distance2)) / float(
                len(self.distance2)
            ))
            self.precisions.append(float(sum(d < threshold for d in self.distance1)) / float(
                len(self.distance1)
            ))

        self.recalls = np.array(self.recalls)
        self.precisions = np.array(self.precisions)
        self.fscores = (1+self.beta**2)*((self.recalls*self.precisions)/((self.beta**2*self.precisions)+self.recalls))

        return self.precisions, self.recalls, self.fscores

    def save_figs(self, label, ax_rec, ax_prec, ax_fscore):
        ax_rec.plot(self.threshold_range, self.recalls, label=label)
        ax_prec.plot(self.threshold_range, self.precisions, label=label)
        ax_fscore.plot(self.threshold_range, self.fscores, label=label)


def main():

    zed = sl.Camera()
    translation = sl.Translation()
    orientation = sl.Orientation()
    translation.init_vector(-4,-75,1.5)
    orientation.init_vector(0.0, 0.0, 1.0, 0.0)
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_params.set_from_svo_file("path/to/svo")
    init_params.svo_real_time_mode = True


    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(err)
        exit(-1)

 
    # Get camera parameters
    camera_parameters = zed.get_camera_information().camera_configuration.calibration_parameters.left_cam

    if CREATE_MESH:
        pymesh = sl.Mesh()              # Current incremental mesh
    else:
        pymesh = sl.FusedPointCloud()   # Current incremental FusedPointCloud
    image = sl.Mat()                    # Left image from camera
    pose = sl.Pose()                    # Camera pose tracking data
   
    viewer = gl.GLViewer()
    viewer.init(camera_parameters, pymesh, CREATE_MESH)

    spatial_mapping_parameters = sl.SpatialMappingParameters()
    spatial_mapping_parameters.resolution_meter = 0.08
    spatial_mapping_parameters.range_meter = 10


   
    tracking_state = sl.POSITIONAL_TRACKING_STATE.OFF
    mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
    mapping_activated = False
    last_call = time.time()             # Timestamp of last mesh request

    # Enable positional tracking
    err = zed.enable_positional_tracking()
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        exit()

    # Set runtime parameters
    runtime = sl.RuntimeParameters()

    print("Press on 'Space' for enable / disable spatial mapping")
    print("Disable the spacial mapping after enabled it will output a .obj mesh file")

    while viewer.is_available():
        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # Update pose data (used for projection of the mesh over the current image)
            tracking_state = zed.get_position(pose)

            if mapping_activated:
                mapping_state = zed.get_spatial_mapping_state()
                # Compute elapsed time since the last call of Camera.request_spatial_map_async()
                duration = time.time() - last_call  
                # Ask for a mesh update if 500ms elapsed since last request
                if(duration > .5 and viewer.chunks_updated()):
                    zed.request_spatial_map_async()
                    last_call = time.time()
                
                if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
                    zed.retrieve_spatial_map_async(pymesh)
                    viewer.update_chunks()
                
            change_state = viewer.update_view(image, pose.pose_data(), tracking_state, mapping_state)

            if change_state:
                if not mapping_activated:
                    init_pose = sl.Transform()
                    init_pose.init_orientation_translation(orientation, translation)
                    zed.reset_positional_tracking(init_pose)

                    # Configure spatial mapping parameters
                    spatial_mapping_parameters.resolution_meter = sl.SpatialMappingParameters().get_resolution_preset(sl.MAPPING_RESOLUTION.LOW)
                    spatial_mapping_parameters.range_meter = sl.SpatialMappingParameters().get_range_preset(sl.MAPPING_RANGE.LONG)
                    spatial_mapping_parameters.use_chunk_only = True
                    spatial_mapping_parameters.save_texture = False         # Set to True to apply texture over the created mesh
                    if CREATE_MESH:
                        spatial_mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.MESH
                    else:
                        spatial_mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD

                    # Enable spatial mapping
                    zed.enable_spatial_mapping(spatial_mapping_parameters)

                    # Clear previous mesh data
                    pymesh.clear()
                    viewer.clear_current_mesh()

                    # Start timer
                    last_call = time.time()

                    mapping_activated = True
                else:
                    # Extract whole mesh
                    zed.extract_whole_spatial_map(pymesh)

                    if CREATE_MESH:
                        filter_params = sl.MeshFilterParameters()
                        filter_params.set(sl.MESH_FILTER.MEDIUM) 
                        # Filter the extracted mesh
                        pymesh.filter(filter_params, True)
                        viewer.clear_current_mesh()

                        # If textures have been saved during spatial mapping, apply them to the mesh
                        if(spatial_mapping_parameters.save_texture):
                            print("Save texture set to : {}".format(spatial_mapping_parameters.save_texture))
                            pymesh.apply_texture(sl.MESH_TEXTURE_FORMAT.RGBA)

                    # Save mesh as an obj file
                    filepath = "test_mesh.obj"
                    status = pymesh.save(filepath)
                    if status:
                        print("Mesh saved under " + filepath)
                    else:
                        print("Failed to save the mesh under " + filepath)
                    
                    mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
                    mapping_activated = False
    
if __name__ == "__main__":

    main()

    test_mesh_path = "path/to/mesh"
    gt_mesh_path = "path/to/gt_mesh"


    test_mesh = o3d.io.read_triangle_mesh(test_mesh_path)
    test_mesh = test_mesh.sample_points_uniformly(number_of_points=100000)

    gt_mesh = o3d.io.read_triangle_mesh(gt_mesh_path)
    gt_mesh = gt_mesh.sample_points_uniformly(number_of_points=100000)
    
    threshold_range = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    Eval = Evaluation(threshold_range)
    Eval.load_meshes(gt_mesh, test_mesh)  # gt_mesh, test_mesh
    Eval.compute_prec_rec()

    ## plot the results

    fig, (ax_rec, ax_prec, ax_fscore) = plt.subplots(3, 1, figsize=(8,12))
    Eval.save_figs('Unity', ax_rec, ax_prec, ax_fscore)

   
    ax_rec.set_xlabel('Threshold')
    ax_rec.set_ylabel('Recall')

    
    ax_prec.set_xlabel('Threshold')
    ax_prec.set_ylabel('Precision')

   
    ax_fscore.set_xlabel('Threshold')
    ax_fscore.set_ylabel('F-score')

    #Show the resulting figure

    plt.show()
 
