import pyrealsense2 as rs

class realsense():
    def __init__(self):
        # Create a pipeline
        self.pipeline = rs.pipeline()

        #Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 60)

        # Start streaming
        self.profile = self.pipeline.start(config)
        s = self.profile.get_device().query_sensors()[1]
        s.set_option(rs.option.exposure, 80)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        self.spat_filter = rs.spatial_filter()          # Spatial    - edge-preserving spatial smoothing
        self.temp_filter = rs.temporal_filter()    # Temporal   - reduces temporal noise

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_rs_frames(self):
        # frames.get_depth_frame() is a 640x480 depth image
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            return (color_frame, aligned_depth_frame)

        self.filtered = self.spat_filter.process(aligned_depth_frame)
        self.filtered = self.temp_filter.process(self.filtered)

        return (color_frame, self.filtered)

    def cord_3d(self, pixel_2d):
        depth_intrin = self.filtered.profile.as_video_stream_profile().intrinsics
        distance = self.filtered.as_depth_frame().get_distance(pixel_2d[0], pixel_2d[1])
        return rs.rs2_deproject_pixel_to_point(depth_intrin, pixel_2d, distance)

    def point_projection(self, pixel_3d):
        depth_intrin = self.filtered.profile.as_video_stream_profile().intrinsics
        return rs.rs2_project_point_to_pixel(depth_intrin, pixel_3d)


    def __del__(self):
        self.pipeline.stop()