#!/bin/bash

echo "🔧 配置 Conda 环境以支持 ROS..."

# 检查当前是否在 conda 环境中
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    echo "❌ 请先激活 conda 环境！"
    echo "   例如: conda activate base"
    exit 1
fi

echo "✅ 当前 Conda 环境: $CONDA_DEFAULT_ENV"

# 安装 ROS Python 依赖
echo "📦 安装 ROS Python 包..."
pip install rospkg catkin_pkg empy defusedxml netifaces

# 设置 ROS 环境变量到 conda 环境
CONDA_PREFIX=$(conda info --base)/envs/$CONDA_DEFAULT_ENV
if [ "$CONDA_DEFAULT_ENV" = "base" ]; then
    CONDA_PREFIX=$(conda info --base)
fi

# 创建 conda 环境激活脚本
ACTIVATE_DIR="$CONDA_PREFIX/etc/conda/activate.d"
DEACTIVATE_DIR="$CONDA_PREFIX/etc/conda/deactivate.d"

mkdir -p "$ACTIVATE_DIR"
mkdir -p "$DEACTIVATE_DIR"

# 激活时加载 ROS 环境
cat > "$ACTIVATE_DIR/ros_env.sh" << 'EOF'
#!/bin/bash
# 保存原始 PYTHONPATH
export _OLD_PYTHONPATH=$PYTHONPATH

# 加载 ROS Noetic 环境
source /opt/ros/noetic/setup.bash

# 确保 conda site-packages 优先
export PYTHONPATH=$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH

echo "[ROS] 已加载到 Conda 环境"
EOF

# 退出时恢复
cat > "$DEACTIVATE_DIR/ros_env.sh" << 'EOF'
#!/bin/bash
if [ -n "$_OLD_PYTHONPATH" ]; then
    export PYTHONPATH=$_OLD_PYTHONPATH
    unset _OLD_PYTHONPATH
fi
EOF

chmod +x "$ACTIVATE_DIR/ros_env.sh"
chmod +x "$DEACTIVATE_DIR/ros_env.sh"

echo ""
echo "✅ 配置完成！"
echo ""
echo "📝 使用方法:"
echo "   1. 重新激活 conda 环境:"
echo "      conda deactivate"
echo "      conda activate $CONDA_DEFAULT_ENV"
echo ""
echo "   2. 测试 ROS 是否工作:"
echo "      python -c 'import rospy; print(\"ROS OK\")'"
echo ""
echo "   3. 运行程序:"
echo "      python main.py --ros"
echo "      python ros_target_publisher.py"
