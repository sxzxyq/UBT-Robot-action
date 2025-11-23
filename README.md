# UBT-Robot-action

## 项目概述
UBT-Robot-action是一个基于ROS 2的机器人动作控制包，主要实现机器人的目标跟踪、运动控制、动作执行及语音播放等功能。适用于需要通过ROS 2控制机器人完成特定交互任务的场景（如目标跟随、挥手、鞠躬等动作）。


## 功能说明
- 目标跟踪：通过订阅目标坐标话题，实现对稳定目标的识别与跟踪
- 运动控制：发布速度指令控制机器人移动，支持旋转到目标方向、距离控制等
- 动作执行：调用机器人动作服务，支持挥手、鞠躬等预设动作
- 语音播放：通过发布语音指令，播放指定路径的音频文件
- 模式切换：支持机器人运动模式切换（如站立、原地踏步等）


## 依赖环境
- ROS 2 (Foxy及以上版本)
- Python 3.8+
- 依赖ROS 2包：
  - `rclpy`
  - `geometry_msgs`
  - `std_msgs`
  - `hric_msgs` (机器人交互相关消息及服务)
  - `ament_copyright`、`ament_flake8`、`ament_pep257` (代码检查工具)
  - `pytest` (测试框架)


## 安装步骤

1. **创建ROS 2工作空间**（若已存在可跳过）：
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **克隆项目代码**：
   ```bash
   git clone https://github.com/sxzxyq/UBT-Robot-action.git  
   ```

3. **编译项目**：
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select action image_inverter_interfaces
   ```

4. **激活工作空间**：
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```


## 快速使用

### 主要节点运行命令

1. **天工机器人主控制器**（目标跟踪+综合控制）：
   ```bash
   ros2 run action tiangong  # 基础版本
   # 或运行其他版本
   ros2 run action tiangong2
   ros2 run action tiangong3
   ```

2. **挥手动作节点**（挥手动作）：
   ```bash
   ros2 run action huishou
   ```

3. **动作测试节点**（测试运动模式切换与动作执行）：
   ```bash
   ros2 run action test_hand
   ```


## 项目结构
```
UBT-Robot-action/
├── src/
│   ├── action/                  # 主功能包
│   │   ├── action/              # 核心代码
│   │   │   ├── tiangong.py      # 基础控制器
│   │   │   ├── tiangong2.py     # 增强版控制器（含验证服务）
│   │   │   ├── tiangong3.py     # 优化版控制器
│   │   │   ├── tiangong3_test.py # 测试版控制器
│   │   │   ├── huishou.py       # 挥手动作节点
│   │   │   └── test_hand.py     # 动作测试节点
│   │   ├── test/                # 测试文件
│   │   │   ├── test_copyright.py # 版权检查测试
│   │   │   ├── test_flake8.py   # 代码风格检查
│   │   │   └── test_pep257.py   # 代码文档检查
│   │   ├── setup.py             # 包配置
│   │   └── package.xml          # ROS包信息
│   └── image_inverter_interfaces/ # 接口定义包
│       ├── srv/                 # 服务定义
│       ├── CMakeLists.txt       # CMake配置
│       └── package.xml          # 接口包信息
└── .gitignore                   # Git忽略文件
```


## 测试
运行代码检查与单元测试：
```bash
# 在工作空间根目录执行
colcon test --packages-select action
# 查看测试结果
colcon test-result --all
```


## 注意事项
1. 语音文件路径：代码中默认语音路径为`/home/nvidia/data/speech/`，需确保该路径下存在对应音频文件（如`tixing1.mp3`、`thanks.mp3`等），或修改代码中的路径为实际音频位置。
2. 依赖服务：机器人运动控制依赖`/hric/motion/set_motion_mode`和`/hric/motion/set_motion_number`服务，请确保机器人驱动节点已启动并提供这些服务。
3. 目标坐标话题：默认订阅`/helmet/position_filtered_transformed`话题获取目标坐标，若实际话题不同，需在代码中修改订阅话题名称。
