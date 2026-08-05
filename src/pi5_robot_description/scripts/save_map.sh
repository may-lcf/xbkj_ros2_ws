#!/bin/bash
# ============================================
# 保存地图脚本（自动编号，永不覆盖）
# ============================================
# 用法:
#   save_map.sh            → 自动命名为 map_01, map_02, ...
#   save_map.sh my_map     → 指定名称为 my_map
#   save_map.sh --dry-run  → 仅显示将使用的文件名，不实际保存
# ============================================

set -euo pipefail

MAP_DIR=~/ros2_ws/src/pi5_robot_description/maps
mkdir -p "$MAP_DIR"

# ---------- 参数解析 ----------
DRY_RUN=false
CUSTOM_NAME=""

for arg in "$@"; do
    case "$arg" in
        --dry-run|-n)
            DRY_RUN=true
            ;;
        --help|-h)
            echo "用法: save_map.sh [地图名称] [--dry-run]"
            echo "  无参数        → 自动编号 (map_01, map_02, ...)"
            echo "  my_map        → 指定名称"
            echo "  --dry-run/-n  → 仅显示文件名，不保存"
            exit 0
            ;;
        *)
            CUSTOM_NAME="$arg"
            ;;
    esac
done

# ---------- 确定地图名称 ----------
if [ -n "$CUSTOM_NAME" ]; then
    # 用户指定了名称
    MAP_NAME="$CUSTOM_NAME"
else
    # 自动编号：扫描已有 map_NN 文件，找到最大编号 +1
    MAX=0
    for f in "$MAP_DIR"/map_[0-9][0-9].*; do
        [ -e "$f" ] || continue
        # 从文件名提取数字部分
        base=$(basename "$f")
        num="${base#map_}"
        num="${num%%.*}"
        # 验证是否为纯数字
        if [[ "$num" =~ ^[0-9]+$ ]]; then
            num=$((10#$num))  # 去掉前导零
            if [ "$num" -gt "$MAX" ]; then
                MAX=$num
            fi
        fi
    done
    NEXT=$((MAX + 1))
    MAP_NAME=$(printf "map_%02d" "$NEXT")
fi

MAP_PATH="$MAP_DIR/$MAP_NAME"

# ---------- 检查文件是否已存在 ----------
if [ -f "$MAP_PATH.yaml" ] || [ -f "$MAP_PATH.pgm" ]; then
    echo "⚠️  文件已存在: $MAP_PATH.*"
    # 如果是自动编号模式，继续递增
    if [ -z "$CUSTOM_NAME" ]; then
        # 重新扫描，从当前 MAX+1 开始找空位
        while [ -f "$MAP_PATH.yaml" ] || [ -f "$MAP_PATH.pgm" ]; do
            NEXT=$((NEXT + 1))
            MAP_NAME=$(printf "map_%02d" "$NEXT")
            MAP_PATH="$MAP_DIR/$MAP_NAME"
        done
        echo "   自动递增到: $MAP_NAME"
    else
        echo "   将覆盖现有文件！"
    fi
fi

# ---------- Dry Run ----------
if [ "$DRY_RUN" = true ]; then
    echo "[dry-run] 将保存到: $MAP_PATH.yaml / .pgm"
    exit 0
fi

# ---------- 保存地图 ----------
echo "========================================="
echo "保存地图到: $MAP_PATH"
echo "========================================="

# ROS2 setup 脚本内部引用未绑定变量，set -u 下会报错，先临时关闭
set +u
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

# 使用 nav2_map_server 的 map_saver_cli 保存地图
ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_PATH" \
    --ros-args -p save_map_timeout:=10.0

RESULT=$?

if [ $RESULT -eq 0 ]; then
    # 获取文件大小
    YAML_SIZE=$(stat -c%s "$MAP_PATH.yaml" 2>/dev/null || echo "?")
    PGM_SIZE=$(stat -c%s "$MAP_PATH.pgm" 2>/dev/null || echo "?")
    # 读取地图尺寸
    MAP_INFO=$(grep -E "^(width|height|resolution):" "$MAP_PATH.yaml" 2>/dev/null || echo "")

    echo ""
    echo "✅ 地图保存成功！"
    echo "-----------------------------------------"
    echo "  YAML: $MAP_PATH.yaml ($YAML_SIZE bytes)"
    echo "  PGM:  $MAP_PATH.pgm ($PGM_SIZE bytes)"
    if [ -n "$MAP_INFO" ]; then
        echo "  信息: $MAP_INFO"
    fi
    echo "-----------------------------------------"
    echo ""
    echo "下次运行 save_map.sh 将保存为: $(printf 'map_%02d' $((NEXT + 1)))"
else
    echo ""
    echo "❌ 地图保存失败！请确保："
    echo "  1. slam_mapping.launch.py 正在运行"
    echo "  2. /map 话题正在发布"
    echo "  3. ROS_DOMAIN_ID 设置正确"
    exit 1
fi
