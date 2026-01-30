#!/bin/bash

# 数据集路径
pathDatasetTUM_VI='/home/slam/workspace/OAK_4_P/ORB_SMAL3/dataset'

# 进入 OpenMAVIS 目录
cd /home/slam/workspace/OAK_4_P/ORB_SMAL3/OpenMAVIS

echo "=========================================="
echo "运行 TUM-VI Corridor 4 数据集"
echo "=========================================="
echo ""
echo "请选择运行模式："
echo "1. 单目 (Monocular)"
echo "2. 双目 (Stereo)"
echo "3. 单目+IMU (Monocular-Inertial)"
echo "4. 双目+IMU (Stereo-Inertial) - 推荐"
echo ""
read -p "请输入选项 (1-4): " choice

case $choice in
    1)
        echo "启动单目模式..."
        ./Examples/Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM-VI.yaml \
            "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam0/data \
            Examples/Monocular/TUM_TimeStamps/dataset-corridor4_512.txt \
            dataset-corridor4_512_mono
        ;;
    2)
        echo "启动双目模式..."
        ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM-VI.yaml \
            "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam0/data \
            "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam1/data \
            Examples/Stereo/TUM_TimeStamps/dataset-corridor4_512.txt \
            dataset-corridor4_512_stereo
        ;;
    3)
        echo "启动单目+IMU模式..."
        ./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM_512.yaml \
            "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam0/data \
            Examples/Monocular-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt \
            Examples/Monocular-Inertial/TUM_IMU/dataset-corridor4_512.txt \
            dataset-corridor4_512_monoi
        ;;
    4)
        echo "启动双目+IMU模式..."
        ./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM-VI.yaml \
            "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam0/data \
            "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam1/data \
            Examples/Stereo-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt \
            Examples/Stereo-Inertial/TUM_IMU/dataset-corridor4_512.txt \
            dataset-corridor4_512_stereoi
        ;;
    *)
        echo "无效选项，退出..."
        exit 1
        ;;
esac

