#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# 数据集路径（脚本所在目录的上一个目录/dataset/exp01/exp01_construction_ground_level）
DATASET_PATH="$(dirname "$SCRIPT_DIR")/dataset/exp01/exp01_construction_ground_level"
TIMESTAMPS_FILE="$DATASET_PATH/timestamps.txt"
OUTPUT_NAME="exp01_construction_ground_level"

# 进入脚本所在目录（OpenMAVIS 目录）
cd "$SCRIPT_DIR"

# 检查时间戳文件是否存在，如果不存在则生成
if [ ! -f "$TIMESTAMPS_FILE" ]; then
    echo "生成时间戳文件..."
    cut -d',' -f1 "$DATASET_PATH/mav0/cam0/data.csv" | tail -n +2 > "$TIMESTAMPS_FILE"
    echo "时间戳文件已生成: $TIMESTAMPS_FILE"
fi

echo "=========================================="
echo "运行 Hilti Challenge 数据集"
echo "数据集路径: $DATASET_PATH"
echo "=========================================="

# 运行 Multi-Inertial SLAM
./Examples/Multi-Inertial/multi_inertial_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Multi-Inertial/HiltiChallenge2022.yaml \
    "$DATASET_PATH" \
    "$TIMESTAMPS_FILE" \
    "$OUTPUT_NAME"