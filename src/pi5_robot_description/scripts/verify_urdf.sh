#!/bin/bash
# 验证URDF脚本
# 用法: ./verify_urdf.sh

echo "=== 验证 Pi5 机器人 URDF ==="

# 设置路径
WORKSPACE_DIR="/home/lcf/ros2arm_ws"
URDF_FILE="$WORKSPACE_DIR/src/pi5_robot_description/urdf/pi5_arm_robot.urdf.xacro"

# 检查文件是否存在
if [ ! -f "$URDF_FILE" ]; then
    echo "错误: URDF文件不存在: $URDF_FILE"
    exit 1
fi

echo "URDF文件: $URDF_FILE"

# 处理xacro
echo "处理xacro文件..."
PROCESSED_URDF="/tmp/pi5_arm_robot.urdf"
xacro "$URDF_FILE" -o "$PROCESSED_URDF"

if [ $? -ne 0 ]; then
    echo "错误: xacro处理失败"
    exit 1
fi

echo "成功生成URDF: $PROCESSED_URDF"

# 验证URDF语法
echo "验证URDF语法..."
check_urdf "$PROCESSED_URDF"

if [ $? -ne 0 ]; then
    echo "错误: URDF语法验证失败"
    exit 1
fi

echo "URDF语法验证通过"

# 生成TF树图
echo "生成TF树图..."
urdf_to_graphiz "$PROCESSED_URDF"
if [ $? -eq 0 ]; then
    echo "TF树图已生成: robot.pdf"
    # 可以用evince或浏览器打开
fi

# 检查关键link和joint
echo ""
echo "=== URDF 结构分析 ==="
echo "Links:"
grep '<link name=' "$PROCESSED_URDF" | sed 's/.*name="\([^"]*\)".*/  - \1/'

echo ""
echo "Joints:"
grep '<joint name=' "$PROCESSED_URDF" | sed 's/.*name="\([^"]*\)".*/  - \1/'

echo ""
echo "=== 关键尺寸验证 ==="
echo "底盘尺寸: 26.8cm x 14.3cm x 9cm"
echo "底盘高度: 12cm (从地面)"
echo "轮子直径: 8cm"
echo "轮子厚度: 4cm"
echo "前后轮距: 17.3cm"
echo "左右轮距中心: 21.3cm"
echo "雷达: 5cm直径 x 5.3cm高"
echo "机械臂位置: x=2.9cm, y=0, z=车身顶部"
echo "雷达位置: x=10.9cm, y=0, z=车身顶部+雷达高/2"

echo ""
echo "=== 验证完成 ==="
echo "如果一切正常，可以运行以下命令在RViz中查看:"
echo "ros2 launch pi5_robot_description view_robot.launch.py"
