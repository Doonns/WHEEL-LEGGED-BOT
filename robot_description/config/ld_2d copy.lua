include "map_builder.lua"  
include "trajectory_builder.lua"  

options = {
  map_builder = MAP_BUILDER,  
  trajectory_builder = TRAJECTORY_BUILDER,  
  map_frame = "map",  -- ��ͼ֡������
  tracking_frame = "imu_link",  -- ����֡������
  published_frame = "base_link",  -- ����֡������
  odom_frame = "odom",  -- ��̼�֡������
  provide_odom_frame = true,  -- �Ƿ��ṩ��̼�֡
  publish_frame_projected_to_2d = false,  -- �Ƿ񷢲�2d��̬
  use_pose_extrapolator = true,  -- �����켣������
  use_odometry = false,  -- �Ƿ�ʹ����̼�
  use_nav_sat = false,  -- �Ƿ�ʹ�õ�������
  use_landmarks = false,  -- �Ƿ�ʹ�õر�
  num_laser_scans = 1,  -- �����״������
  num_multi_echo_laser_scans = 0,  -- ��ز������״������
  num_subdivisions_per_laser_scan = 2,  -- ÿ������ɨ���ϸ������
  num_point_clouds = 0,  -- ���Ƶ�����
  lookup_transform_timeout_sec = 0.2,  -- ���ұ任�ĳ�ʱʱ�䣨�룩
  submap_publish_period_sec = 0.3,  -- �ӵ�ͼ�������ڣ��룩
  pose_publish_period_sec = 10e-3,  -- ��̬�������ڣ��룩
  trajectory_publish_period_sec = 50e-3,  -- �켣�������ڣ��룩
  rangefinder_sampling_ratio = 1.,  -- ����ǲ�������
  odometry_sampling_ratio = 1.,  -- ��̼Ʋ�������
  fixed_frame_pose_sampling_ratio = 1.,  -- �̶�֡��̬��������
  imu_sampling_ratio = 1.,  -- IMU��������
  landmarks_sampling_ratio = 1.,  -- �ر��������
}
 
MAP_BUILDER.use_trajectory_builder_2d = true  -- �Ƿ�����2D SLAM
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- 2D�켣���������ӵ�ͼ�ķ�Χ��������
TRAJECTORY_BUILDER_2D.min_range = 0.2  -- �������״���Сɨ�跶Χ���Ȼ����˰뾶С�Ķ�����
TRAJECTORY_BUILDER_2D.max_range = 10  -- �������״����ɨ�跶Χ
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.  -- �������״����ɨ�跶Χ
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- �Ƿ�ʹ��IMU����
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- �Ƿ�ʹ��ʵʱ�ػ����ɨ��ƥ��
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 15.0  -- �������ٶ�ʱ�䳣������

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 1.0�ĳ�0.1,��߶��˶������ж�
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 0.55�ĳ�0.65,Fast csm����ͷ��������ڴ˷����Ž����Ż���
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8  --0.6�ĳ�0.7,ȫ�ֶ�λ��С���������ڴ˷�������ΪĿǰȫ�ֶ�λ��׼ȷ

return options
