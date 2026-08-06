#!/bin/bash
# ============================================================
# 多点导航控制脚本
# 用法: ./waypoint_control.sh <命令>
#
# 命令:
#   start   - 开始多点导航
#   pause   - 暂停导航
#   resume  - 恢复导航
#   cancel  - 取消导航
#   clear   - 清空目标点列表
#   list    - 显示目标点列表
# ============================================================

CMD=${1:-"help"}

if [ "$CMD" = "help" ] || [ "$CMD" = "-h" ] || [ "$CMD" = "--help" ]; then
    echo "=========================================="
    echo "  多点导航控制脚本"
    echo "=========================================="
    echo ""
    echo "用法: $0 <命令>"
    echo ""
    echo "命令:"
    echo "  start   - 开始多点导航（需先标记目标点）"
    echo "  pause   - 暂停当前导航"
    echo "  resume  - 恢复暂停的导航"
    echo "  cancel  - 取消导航并停止"
    echo "  clear   - 清空所有目标点"
    echo "  list    - 显示当前目标点列表"
    echo ""
    echo "使用流程:"
    echo "  1. 在 RViz 中选择 'Publish Point' 工具"
    echo "  2. 依次点击地图标记目标点"
    echo "  3. 运行: $0 start"
    echo "=========================================="
    exit 0
fi

echo "发送命令: $CMD"
ros2 topic pub --once /waypoint_cmd std_msgs/String "data: '$CMD'"
