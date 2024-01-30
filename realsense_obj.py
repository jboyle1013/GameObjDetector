"""
This is the code we are currently using for the RealSense Camera in the objdetclass.py file.
The DepthCamera class encapsulates the necessary methods for camera setup,
streaming, frame processing, and advanced configuration.
"""


import json
import numpy as np
import pyrealsense2 as rs

DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", "0B3A", "0B5C"]

class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.vtx = None
        self.pc = None
        self.advnc_mode = None
        self.clipping_distance = None
        self.clipping_distance_in_meters = None
        self.depth_scale = None
        self.depth_sensor = None
        self.profile = None
        self.depth_intrinsics = None
        self.depth_profile = None
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)
        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        device = self.pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.decimation_filter = rs.decimation_filter()

        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.configure_Filters()
        # Create a colorizer object
        self.colorizer = rs.colorizer()
        # Enable histogram equalization
        self.colorizer.set_option(rs.option.histogram_equalization_enabled, 1)


    def start_Streaming(self):
        # Start streaming
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_sensor.set_option(rs.option.visual_preset, 3)
        # Configure Stream Settings
        self.colorizer.set_option(rs.option.visual_preset, 2)
        self.colorizer.set_option(rs.option.histogram_equalization_enabled, 1.0)

        self.colorizer.set_option(rs.option.color_scheme, 0.0)
        #
        # # Set min and max depth distance (in meters)
        min_distance = 0.0
        max_distance = 1.0
        self.colorizer.set_option(rs.option.min_distance, min_distance)
        self.colorizer.set_option(rs.option.max_distance, max_distance)


    def configure_frame(self):
        # Get stream profile and camera intrinsics
        self.profile = self.pipeline.get_active_profile()
        sensors = self.pipeline.get_active_profile().get_device().sensors
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = self.depth_profile.get_intrinsics()

        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.clipping_distance_in_meters = 0.5 #1/2 meter
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale


    def get_frame(self):
        try:
            self.configure_frame()

            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            depth_frame = self.apply_Filters(depth_frame)
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colorframe = self.colorizer.colorize(depth_frame)
            depth_colormap = np.asanyarray(depth_colorframe.get_data())

            if not depth_frame or not color_frame:
                return False, None, None
            return True, depth_image, color_image, depth_colormap, depth_frame

        except Exception as e:
            print(f"An error occurred in getVal: {e}")

    def deproject(self, pixel, depth):

        return rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, pixel, depth)


    def release(self):
        self.pipeline.stop()


    def get_imu_data(self):
        frames = self.pipeline.wait_for_frames()
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)
        if accel_frame and gyro_frame:
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            return accel_data, gyro_data
        return None, None


    def apply_Filters(self, depth_frame):
        #depth_frame = self.decimation_filter.process(depth_frame)
        depth_frame = self.spatial_filter.process(depth_frame)
        depth_frame = self.temporal_filter.process(depth_frame)

        return depth_frame

    def configure_Filters(self):
        self.decimation_filter.set_option(rs.option.filter_magnitude, 1)  # Reduce the resolution by half
        self.spatial_filter.set_option(rs.option.filter_magnitude, 2)  # The filter strength
        self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.42)  # Spatial filter smooth alpha
        self.spatial_filter.set_option(rs.option.filter_smooth_delta, 1)  # Spatial filter smooth delta
        self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0)
        self.temporal_filter.set_option(rs.option.filter_smooth_delta, 100)

    def find_advanced_mode(self) :
        ctx = rs.context()
        ds5_dev = rs.device()
        devices = ctx.query_devices();
        for dev in devices:
            if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                if dev.supports(rs.camera_info.name):
                    print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
                return dev
        raise Exception("No D400 product line device that supports advanced mode was found")


    def get_Settings(self):

        dev = self.find_advanced_mode()
        self.advnc_mode = rs.rs400_advanced_mode(dev)
        print("Advanced mode is", "enabled" if self.advnc_mode.is_enabled() else "disabled")
        # Get each control's current value
        print("Depth Control: \n", self.advnc_mode.get_depth_control())
        print("RSM: \n", self.advnc_mode.get_rsm())
        print("RAU Support Vector Control: \n", self.advnc_mode.get_rau_support_vector_control())
        print("Color Control: \n", self.advnc_mode.get_color_control())
        print("RAU Thresholds Control: \n", self.advnc_mode.get_rau_thresholds_control())
        print("SLO Color Thresholds Control: \n", self.advnc_mode.get_slo_color_thresholds_control())
        print("SLO Penalty Control: \n", self.advnc_mode.get_slo_penalty_control())
        print("HDAD: \n", self.advnc_mode.get_hdad())
        print("Color Correction: \n", self.advnc_mode.get_color_correction())
        print("Depth Table: \n", self.advnc_mode.get_depth_table())
        print("Auto Exposure Control: \n", self.advnc_mode.get_ae_control())
        print("Census: \n", self.advnc_mode.get_census())

        #To get the minimum and maximum value of each control use the mode value:
        query_min_values_mode = 1
        query_max_values_mode = 2
        current_std_depth_control_group = self.advnc_mode.get_depth_control()
        min_std_depth_control_group = self.advnc_mode.get_depth_control(query_min_values_mode)
        max_std_depth_control_group = self.advnc_mode.get_depth_control(query_max_values_mode)
        print("Depth Control Min Values: \n ", min_std_depth_control_group)
        print("Depth Control Max Values: \n ", max_std_depth_control_group)

    def export_Settings_to_json(self):

        dev = self.find_advanced_mode()
        advnc_mode = rs.rs400_advanced_mode(dev)
        print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")
        serialized_string = advnc_mode.serialize_json()
        print("Controls as JSON: \n", serialized_string)
        as_json_object = json.loads(serialized_string)
        with open('camerasettings/camsettings.json', 'w') as file:
            # write  to the file
            json.dump(as_json_object, file, indent=4)

    def set_Settings_from_json(self, json_path):
        print("[INFO] Configuring Camera...")
        dev = self.find_advanced_mode()
        self.advnc_mode = rs.rs400_advanced_mode(dev)
        try:
            with open(json_path, 'r') as file:
                json_string = file.read()
            settings = json_string.replace("'", '\"')
            self.advnc_mode.load_json(settings)
        except FileNotFoundError:
            print(f"Error: The file {json_path} was not found.")
        except Exception as e:
            print(f"An error occurred: {e}")
