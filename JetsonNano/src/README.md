## metrobot-ros
+ Repository for Metrobot packages for ROS Melodic, meant for running on the Jetson Nano (Ubuntu 18.04).

+ Usage
    + `roslaunch realsense2_camera rs_d435i_and_t265.launch`
      + ↑ included in: `roslaunch localization localization.launch`
    
  + Summary of topics that may be useful:
    + `/d435i`
      + `/color/image_raw`
      + `/aligned_depth_to_color/image_raw`
      + `/depth`
        + `/color/points`
        + `/image_rect_raw`
    + `/t265/odom/sample`
    + camera info K Matrix:
      + `camera2_align_depth` = true:
        + Use `/d435i/aligned_depth_to_color/camera_info`, same as `/d435i/color/camera_info`:
          + 1280*720: [921.568359375, 0.0, 643.44873046875, 0.0, 919.7422485351562, 379.1036071777344, 0.0, 0.0, 1.0]
          + 848*480: [614.37890625, 0.0, 426.29913330078125, 0.0, 613.1614990234375, 252.73573303222656, 0.0, 0.0, 1.0]
          + 640*480: [614.37890625, 0.0, 322.2991638183594, 0.0, 613.1614990234375, 252.73573303222656, 0.0, 0.0, 1.0]
          + 640*360: [460.7841796875, 0.0, 321.724365234375, 0.0, 459.8711242675781, 189.5518035888672, 0.0, 0.0, 1.0]
          + 424*240: [307.189453125, 0.0, 213.14956665039062, 0.0, 306.58074951171875, 126.36786651611328, 0.0, 0.0, 1.0]
      + `camera2_align_depth` = false:
        + Use `/d435i/depth/camera_info`:
          + 1280*720: [642.2916870117188, 0.0, 637.2142944335938, 0.0, 642.2916870117188, 362.7745361328125, 0.0, 0.0, 1.0]
          + 848*480: [425.51824951171875, 0.0, 422.15447998046875, 0.0, 425.51824951171875, 241.838134765625, 0.0, 0.0, 1.0]
          + 640*480: [385.375, 0.0, 318.3285827636719, 0.0, 385.375, 241.6647186279297, 0.0, 0.0, 1.0]
          + 640*360: [321.1458435058594, 0.0, 318.6071472167969, 0.0, 321.1458435058594, 181.38726806640625, 0.0, 0.0, 1.0]
          + 424*240: [212.75912475585938, 0.0, 211.07723999023438, 0.0, 212.75912475585938, 120.9190673828125, 0.0, 0.0, 1.0]
      
  
  + Depth Scaling: depth img -> depth in meters:
    + pix val * 0.001 = dis in meters
    + known from global search `_depth_scale_meters = sensor.as<rs2::depth_sensor>().get_depth_scale();` and output the scale
  
  + change of rgbd point cloud frame with `align_depth`:
    + `align_depth=true`: `d435i_color_optical_frame`
    + `align_depth=false`: `d435i_depth_optical_frame`
  
  + TF Frames:
    + `/t265_odom_frame`: the world frame, static, initialized by T265 from static
    + `/d435i_color_optical_frame`: can directly look up
    + `/ball_real`
  
  + openCV tips:
    + cv::Mat type check
      + info, take `CV_8UC3` as eg:
        + `8U`: 8 bits per int, unsigned, if it's `S` then it's signed
        + `C3`: 3 channels
      + image.depth: returns the number corresponding to the bits-per-int and signed/unsigned config
        + `CV_8U`, `CV_8S`, ... all have unique nums corresponding to them
      + image.channels: num of channels, CV_8UC3 has 3 channels
  
  + Set Dynamic Reconfigure Parameter via cmd line:
    + list all the configurable nodes(equiv to those shown in left bar of rqt_reconfigure GUI): `rosrun dynamic_reconfigure dynparam list`
    + `rosrun dynamic_reconfigure dynparam set </node> <param_name> <val>`
    + auto exposure: `rosrun dynamic_reconfigure dynparam set /d435i/rgb_camera enable_auto_exposure false`
    + rgb exposure rate: `rosrun dynamic_reconfigure dynparam set /d435i/rgb_camera exposure 166`
    
  + CV Threshold:
    + HSV:
      + tennis ball:
        + at Warren's home:
          + | Warren's Home, sunny                | H_MIN | H_MAX | S_MIN | S_MAX | V_MIN | V_MAX |
            |-------------------------------------|-------|-------|-------|-------|-------|-------|
            | morning                             |   78  |   96  |   68  |  164  |  39   |  223  |
            | late afternoon OR elec light        |   78  |   99  |   86  |  164  |  62   |  233  |

        + at lab
          + | Lab, sunny,                         | H_MIN | H_MAX | S_MIN | S_MAX | V_MIN | V_MAX |
            |-------------------------------------|-------|-------|-------|-------|-------|-------|
            | donot look at walls, exp 300        |   76  |   95  |  68   |  147  |  78   |  214  |
            | bright ball, no wall, exp 350☆, eve |   67  |   85  |  77   |  188  |  77   |  256  |
      + green cup:
        + | Lab                            | H_MIN | H_MAX | S_MIN | S_MAX | V_MIN | V_MAX |
          |--------------------------------|-------|-------|-------|-------|-------|-------|
          | exp 900                        |   34  |   61  |  164  |  256  |  78   |  214  |
          | auto exp                    x  |   41  |   73  |  104  |  256  |  35   |  140  |
      + green tennis ball:
          + | Lab                            | H_MIN | H_MAX | S_MIN | S_MAX | V_MIN | V_MAX |
            |--------------------------------|-------|-------|-------|-------|-------|-------|
            | exp 1000                       |   55  |   85  |  79   |  150  |  24   |  256  |
            | exp 800                        |   55  |   85  |  67   |  150  |  39   |  256  |
            | exp 700                    ☆   |   55  |   85  |  67   |  150  |  39   |  256  |
        + at Michael's Home
            + | Green ball, AutoExp                 | H_MIN | H_MAX | S_MIN | S_MAX | V_MIN | V_MAX |
              |-------------------------------------|-------|-------|-------|-------|-------|-------|
              | with spotlight off                  |   54  |   89  |   80  |  191  |  40   |  256  |
  
  + Ball Estimation Params:
    + ************** At warren's home *********************
      + ------------6Hz, can't capsture shooting spd----------------
      ```
      <arg name="d435i_lin_vel_est_max_his_time" default="0.3"/> <!-- if fps=6Hz, set this to >0.3s(due to -frame_dt), 0.05 will be too noisy -->
      
      <arg name="ball_est_max_recent_his_time" default="0.3"/>  <!-- if fps=6Hz, set this to >0.3s(due to -frame_dt) -->
      <arg name="ball_est_max_lost_time" default="1"/>
      <arg name="ball_est_noise_overcoming_vel" default="0.2"/>
      <arg name="ball_est_short_window_time" default="0.2"/> <!-- should > 1/fps -->
      ```
      + ---------15 Hz, can capture a bit of shooting spd-------------------------
      ```
      <arg name="d435i_lin_vel_est_max_his_time" default="0.2"/> <!-- if fps=6Hz, set this to >0.3s(due to -frame_dt), 0.05 will be too noisy -->
      
      <arg name="ball_est_max_recent_his_time" default="0.25"/>  <!-- if fps=6Hz, set this to >0.3s(due to -frame_dt) -->
      <arg name="ball_est_max_lost_time" default="1"/>
      <arg name="ball_est_noise_overcoming_vel" default="0.2"/>
      <arg name="ball_est_short_window_time" default="0.1"/> <!-- should > 1/fps -->
      ```
      + ---------60Hz, can capture shoot vel------------
      ```
      <arg name="d435i_lin_vel_est_max_his_time" default="0.1"/> <!-- if fps=6Hz, set this to >0.5s, 0.05 will be too noisy -->
      
      <arg name="ball_est_max_recent_his_time" default="0.25"/>  <!-- if fps=6Hz, set this to >0.4s -->
      <arg name="ball_est_max_lost_time" default="1"/>
      <arg name="ball_est_noise_overcoming_vel" default="0.2"/>
      <arg name="ball_est_short_window_time" default="0.1"/> <!-- should > 2*(1/fps) -->
      ```
  
  + resolution & morph:
    + 848*480:
    ```
    <arg name="erode_diam" default="4"/> <!--pix elements smaller than this will be eroded-->
    <arg name="dilate_diam" default="15"/> <!--pix elements larger than this will be dilated-->
    <arg name="min_obj_pix_diam" default="15"/> <!-- MIN_OBJECT_AREA = min_obj_pix_diam^2 -->
    ```
    + 424*240:
    ```
    <arg name="erode_diam" default="4"/> <!--pix elements smaller than this will be eroded-->
    <arg name="dilate_diam" default="10"/> <!--pix elements larger than this will be dilated-->
    <arg name="min_obj_pix_diam" default="10"/> <!-- MIN_OBJECT_AREA = min_obj_pix_diam^2 -->
    ```