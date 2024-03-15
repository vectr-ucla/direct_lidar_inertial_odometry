###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#                                                         #
###########################################################

/**:
  ros__parameters:
    use_sim_time: true
    frames/odom: odom
    frames/baselink: base_link
    frames/lidar: lidar
    frames/imu: imu

    map/waitUntilMove: true
    map/dense/filtered: false
    map/sparse/leafSize: 0.25

    odom/gravity: 9.80665
    odom/computeTimeOffset: true

    odom/imu/approximateGravity: false
    odom/imu/calibration/gyro: true
    odom/imu/calibration/accel: true
    odom/imu/calibration/time: 3.0
    odom/imu/bufferSize: 5000

    odom/preprocessing/cropBoxFilter/size: 1.0
    odom/preprocessing/voxelFilter/res: 0.25

    odom/keyframe/threshD: 1.0
    odom/keyframe/threshR: 45.0

    odom/submap/keyframe/knn: 10
    odom/submap/keyframe/kcv: 10
    odom/submap/keyframe/kcc: 10
    odom/gicp/minNumPoints: 64
    odom/gicp/kCorrespondences: 16
    odom/gicp/maxCorrespondenceDistance: 0.5
    odom/gicp/maxIterations: 32
    odom/gicp/transformationEpsilon: 0.01
    odom/gicp/rotationEpsilon: 0.01
    odom/gicp/initLambdaFactor: 1e-9

    odom/geo/Kp: 4.5
    odom/geo/Kv: 11.25
    odom/geo/Kq: 4.0
    odom/geo/Kab: 2.25
    odom/geo/Kgb: 1.0
    odom/geo/abias_max: 5.0
    odom/geo/gbias_max: 0.5
