# ESP32 智能小车 - 基于 MicroROS 自主导航平台

一款基于 ESP32 的开源差速智能小车，集成 MicroROS 通讯、里程计、虚拟激光雷达、姿态控制、OLED 可视化，支持 ROS2 导航功能。

## 功能特性

### 🔥 核心功能
- **双电机闭环调速**（PWM 调速 / 差速转向）
- **舵机旋转简易激光雷达**（舵机旋转测距，发布 /scan 点云）
- **OLED 实时状态显示**（速度、电量、模式、IP）
- **陀螺仪姿态融合**（MPU6050 提供 IMU 数据）
- **MicroROS 通讯**（与 ROS2 实时交互）
- **完整导航支持**（可直接运行 SLAM + 自主导航）

### 📡 ROS2 接口
#### 发布话题
- `/odom` —— 里程计（位置 + 速度）
- `/scan` —— 简易激光雷达数据（舵机旋转）
- `/imu` —— 陀螺仪姿态数据

#### 订阅话题
- `/cmd_vel` —— 接收运动控制指令（teleop 键盘/手柄控制）

#### 支持导航功能
- 建图（SLAM）
- 自主导航（Navigation2）
- 避障
- 定点巡航

## 硬件清单
- ESP32 主控
- 电机驱动模块（L298N/A4950/DRV8833）
- 减速电机 x2
- 舵机 SG90
- 超声波/TOF 测距模块
- MPU6050 陀螺仪
- OLED 0.96寸 I2C
- 电源模块

## 使用方法

### 1. 编译上传
使用 PlatformIO 或 Arduino 编译并烧录到 ESP32。

### 2. 运行 MicroROS 代理
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
3. 键盘控制小车
bash
运行
ros2 run teleop_twist_keyboard teleop_twist_keyboard
4. 查看雷达 + 里程计
bash
运行
rviz2
5. 启动导航
支持直接运行 Navigation2 进行自主导航。
控制说明
前进 / 后退 / 左转 / 右转
速度可调
舵机激光雷达 360° 扫描
自动避障
OLED 实时显示系统状态
适用场景
ROS2 入门学习
自主导航小车
SLAM 建图
机器人避障算法验证
大中专 / 高校机器人实训
作者
基于 ESP32 + MicroROS 智能小车平台

### 📜 开源声明 & 第三方组件版权说明
1.  本项目**自定义业务源码**基于 **Apache 2.0** 协议开源，仅用于个人学习、嵌入式机器人技术研究。
2.  项目依赖所有第三方开源组件版权归原作者所有：
    - STM32 HAL库/CMSIS内核：版权归 STMicroelectronics 官方所有
    - FreeRTOS：版权归 Amazon Real-Time Engineers 所有，基于MIT协议
    - ROS2 Humble 生态组件：基于 Apache 2.0 开源协议
    - OpenCV、nlohmann_json：基于原生开源宽松协议
    - YOLOv8：版权归 Ultralytics 所有，本项目仅用于**个人非商用学习研究**
    -还有许多不一一注明
3.  所有底层库、官方固件仅作为开发依赖调用，未修改原生版权声明，不用于任何商业盈利行为。
4.  禁止将本项目全部/部分源码用于未经授权的商业售卖、产品商用。
5. ## 📜 开源来源声明
   本项目底层基础框架、ESP32 MicroROS 底层架构、小车基础驱动源码来源于 **鱼香ROS** 开源项目，基于 **MIT 协议**二次开发。

