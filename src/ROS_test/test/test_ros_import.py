"""测试 ROS 模块是否可以导入"""

import sys

print(f"🐍 Python 路径: {sys.executable}")
print(f"📦 Python 版本: {sys.version}")
print()

# 测试导入
modules = ['rospy', 'rospkg', 'std_msgs', 'geometry_msgs']

for module in modules:
    try:
        exec(f"import {module}")
        print(f"✅ {module:20s} - OK")
    except ImportError as e:
        print(f"❌ {module:20s} - FAILED: {e}")

print()
print("测试完成！")
