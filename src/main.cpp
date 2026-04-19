#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>




// ROS2核心头文件
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/timer.h>
#include <rosidl_runtime_c/string_functions.h>
#include <builtin_interfaces/msg/time.h>

// TF 使用 geometry_msgs 原生支持
#include <geometry_msgs/msg/transform_stamped.h>

// 消息类型
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>

// 外设库
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <Adafruit_PWMServoDriver.h>

// 电机/运动学
#include "Esp32PcntEncoder.h"
#include "Esp32McpwmMotor.h"
#include "PidController.h"
#include "Kinematics.h"

#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X vl53;

// ==================== 基础配置 ====================
#define WIFI_SSID           "Galaxy M30sF6AF"
#define WIFI_PASSWORD       "123456px"
#define AGENT_IP            "192.168.43.212"
#define AGENT_PORT          8888
#define M_PI                3.14159265358979323846

#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT       64
#define OLED_RESET          -1
#define SCREEN_ADDR         0x3C
#define I2C_SDA             18
#define I2C_SCL             19

#define Trig                17
#define Echo                21
#define PCA9685_ADDR        0x40
#define SERVO_MIN_PULSE     500
#define SERVO_MAX_PULSE     2000

#define MOTOR0_A_GPIO       22
#define MOTOR0_B_GPIO       23
#define MOTOR1_A_GPIO       12
#define MOTOR1_B_GPIO       13
#define ENCODER0_A_GPIO     32
#define ENCODER0_B_GPIO     33
#define ENCODER1_A_GPIO     26
#define ENCODER1_B_GPIO     25
// 原来的参数是针对mm/s的，现在m/s要放大1000倍
#define PID_KP              625.0f
#define PID_KI              125.0f
#define PID_KD              0.0
// #define PID_KP              0.625
// #define PID_KI              0.125
// #define PID_KD              0.0
#define PID_OUT_LIMIT       100
#define ENCODER_PPR         1942
#define WHEEL_DIAMETER      0.065f
#define WHEEL_BASE          0.175f

#define SIMULATION_MODE     false


// 雷达参数
#define NUM_POINTS 121
// 双缓冲：两个数组
float buffer_a[NUM_POINTS]; // 缓冲区 A
float buffer_b[NUM_POINTS]; // 缓冲区 B
// 指针：指向当前“用于发布”的缓冲区
float *publish_buffer = buffer_a; 
// 指针：指向当前“正在采集”的缓冲区
float *capture_buffer = buffer_b;
// 标志位：告诉发布线程“有新一帧数据了”
volatile bool new_data_ready = false;


// ==================== 全局对象 =====================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// TF
rcl_publisher_t tf_pub;
geometry_msgs__msg__TransformStamped tf_msg;

// 发布器/订阅器
rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;
rcl_publisher_t scan_pub;
sensor_msgs__msg__LaserScan scan_msg;
float laser_ranges[121];
rcl_subscription_t twist_sub;
geometry_msgs__msg__Twist twist_msg;

// 定时器
rcl_timer_t odom_timer;
rcl_timer_t scan_timer;
rcl_timer_t motor_timer;

// 硬件
Adafruit_SSD1306 display{SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET};
MPU6050 mpu{Wire};
Adafruit_PWMServoDriver pca9685{PCA9685_ADDR};
PidController pidController[2];
Esp32PcntEncoder encoders[2];
Esp32McpwmMotor motor;
Kinematics kinematics;

float target_motor_speed1 = 0.0f;
float target_motor_speed2 = 0.0f;
bool ros_connected = false;

// ==================== 时间同步函数（核心！）=====================
void set_ros_time(builtin_interfaces__msg__Time *time) {
    // 🔥 这一行是 SLAM 能工作的关键：从 micro-ROS agent 获取系统时间
    int64_t epoch_ns = rmw_uros_epoch_nanos();
    time->sec = epoch_ns / 1000000000;
    time->nanosec = epoch_ns % 1000000000;

}
// ==================== 时间同步函数 =====================
// void set_ros_time(builtin_interfaces__msg__Time *time) {
//     // 直接用 ESP32 开机毫秒数
//     uint64_t ms = millis();
//     time->sec = ms / 1000;
//     time->nanosec = (ms % 1000) * 1000000;
// }




// ==================== 工具函数 ====================
void quaternion_from_euler(float roll, float pitch, float yaw, geometry_msgs__msg__Quaternion* q) {
  float cy = cos(yaw * 0.5); float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5); float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5); float sr = sin(roll * 0.5);
  q->w = cr * cp * cy + sr * sp * sy;
  q->x = sr * cp * cy - cr * sp * sy;
  q->y = cr * sp * cy + sr * cp * sy;
  q->z = cr * cp * sy - sr * sp * cy;
}

void set_ros_string(rosidl_runtime_c__String *ros_str, const char *c_str) {
  if (ros_str->data == NULL) rosidl_runtime_c__String__init(ros_str);
  rosidl_runtime_c__String__assign(ros_str, c_str);
}



void set_servo_angle(int angle) {
  angle = constrain(angle, 0, 120);
  uint16_t pulse_us = map(angle, 0, 120, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  uint16_t pwm = (pulse_us * 4096) / 20000;
  pca9685.setPWM(0, 0, pwm);
  // delay(10);
}

// float old_chaoshengbo_ok_read_ultrasonic() {
//   digitalWrite(Trig, LOW);
//   delayMicroseconds(2);
//   digitalWrite(Trig, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(Trig, LOW);

//   unsigned long mtime = pulseIn(Echo, HIGH, 5000);
//   if (mtime == 0) return 1.0f;

//   float dist = mtime / 58.0 / 100.0;
//   return constrain(dist, 0.1f, 4.0f);
// }

float read_ultrasonic() {
  // 🔥 超声波代码全部注释/删除，换成 VL53L0X
  VL53L0X_RangingMeasurementData_t measure;
  vl53.rangingTest(&measure, false);

  if (measure.RangeStatus == 4) {
    return 2.0f; // 超出范围返回 2m
  }

  float dist = measure.RangeMilliMeter / 1000.0f; // 转成米
  return constrain(dist, 0.03f, 2.0f);
}



float old_get_distance(int angle) {
  set_servo_angle(angle);
  delay(15); //15
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  unsigned long mtime = pulseIn(Echo, HIGH, 5000);
  if (mtime == 0) return 1.0f;
  float dist = mtime / 58.0 / 100.0;
  return constrain(dist, 0.1f, 4.0f);
}

// ==================== 回调函数 =====================
void twist_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_x = msg->linear.x ;
  float angular_z = msg->angular.z;
  kinematics.kinematic_inverse(linear_x, angular_z, target_motor_speed1, target_motor_speed2);
  pidController[0].update_target(target_motor_speed1);
  pidController[1].update_target(target_motor_speed2);
}

void motor_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (!ros_connected) return;
  kinematics.update_motor_ticks(micros(), encoders[0].getTicks(), encoders[1].getTicks());
  float out1 = pidController[0].update(kinematics.motor_speed(0));
  float out2 = pidController[1].update(kinematics.motor_speed(1));
  motor.updateMotorSpeed(0, out1);
  motor.updateMotorSpeed(1, out2);
  
  // 加这行打印，看核心数据
  Serial.printf("target:%.2f | actual:%.2f | out:%.2f | tick:%d\n", 
    target_motor_speed1, kinematics.motor_speed(0), out1, encoders[0].getTicks());
}




void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (!ros_connected) return;

  // 🔥 使用系统时间（时间同步）           
  set_ros_time(&odom_msg.header.stamp);
  set_ros_string(&odom_msg.header.frame_id, "odom");
  set_ros_string(&odom_msg.child_frame_id, "base_footprint");

  odom_t odom = kinematics.odom();
  odom_msg.pose.pose.position.x = odom.x ;
  odom_msg.pose.pose.position.y = odom.y ;
  quaternion_from_euler(0, 0, odom.yaw, &odom_msg.pose.pose.orientation);
  odom_msg.twist.twist.linear.x = odom.linear_speed ;
  odom_msg.twist.twist.angular.z = odom.angular_speed;

  rcl_publish(&odom_pub, &odom_msg, NULL);

  // TF
  // set_ros_time(&tf_msg.header.stamp);
  // set_ros_string(&tf_msg.header.frame_id, "odom");
  // set_ros_string(&tf_msg.child_frame_id, "base_footprint");
  // tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
  // tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
  // tf_msg.transform.translation.z = 0.0;
  // tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
  // rcl_publish(&tf_pub, &tf_msg, NULL);
}

void scan_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (!ros_connected) return;

  static bool init = false;
  if (!init) {
    // 初始化时先随便指一个，后面会更新
    scan_msg.ranges.data = buffer_a; 
    scan_msg.ranges.size = NUM_POINTS;
    scan_msg.ranges.capacity = NUM_POINTS;
    // ... 其他初始化不变 ...
    init = true;
  }

  // 🔥 核心逻辑：如果有新数据，就交换缓冲区指针
  if (new_data_ready) {
    // 交换指针 (原子操作，极快)
    float *temp = publish_buffer;
    publish_buffer = capture_buffer;
    capture_buffer = temp;
    
    new_data_ready = false; // 清除标志位
  }

  // 配置消息头
  set_ros_time(&scan_msg.header.stamp);
  set_ros_string(&scan_msg.header.frame_id, "laser_link");

  // 配置雷达参数 (保持原样)
  scan_msg.angle_min = 0.0;
  scan_msg.angle_max = 2.094;
  scan_msg.angle_increment = 0.01745;
  // 🔥 注意：这里的 scan_time 建议改成你实际扫一圈的时间
  // 比如你现在一圈需要 0.5秒，这里就填 0.5
  scan_msg.scan_time = 0.5; 
  scan_msg.range_min = 0.08;
  scan_msg.range_max = 5.0;

  // 🔥 直接挂指针发布，瞬间完成
  scan_msg.ranges.data = publish_buffer;
  
  rcl_publish(&scan_pub, &scan_msg, NULL);
}



// void old_scan_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
//   if (!ros_connected) return;

//   static bool init = false;
//   if (!init) {
//     scan_msg.ranges.data = laser_ranges;
//     scan_msg.ranges.size = 121;
//     scan_msg.ranges.capacity = 121;
//     scan_msg.intensities.data = NULL;
//     scan_msg.intensities.size = 0;
//     scan_msg.intensities.capacity = 0;
//     init = true;
//   }

//   // 时间同步（正确）
//   set_ros_time(&scan_msg.header.stamp);
  
//   // 🔥 🔥 🔥 【关键：必须用 base_scan 不是 laser_link！】
//   set_ros_string(&scan_msg.header.frame_id, "laser_link");

//   scan_msg.angle_min = 0.0;
//   scan_msg.angle_max = 2.094;    // 120度
//   scan_msg.angle_increment = 0.01745;  // 1度
//   scan_msg.time_increment = 0.0001;
//   scan_msg.scan_time = 0.1;
//   scan_msg.range_min = 0.08;
//   scan_msg.range_max = 5.0;

//   // 填充雷达数据
//   for (int i = 0; i <= 120; i++) {
//     laser_ranges[i] = get_distance(i);
//     // laser_ranges[i] = 4.5;

//   }
  
//   rcl_publish(&scan_pub, &scan_msg, NULL);
// }

// ==================== setup =====================
void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) delay(500);

  IPAddress agent_ip; agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, agent_ip, AGENT_PORT);
  delay(3000);

  // 🔥 时间同步初始化（必须！）
  rmw_uros_sync_session(1000);
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_robot", "", &support);

  // TF
  rclc_publisher_init_default(&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped), "/tf");
  rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  rclc_publisher_init_default(&scan_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
  rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

  rclc_executor_init(&executor, &support.context, 8, &allocator);
  rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, twist_callback, ON_NEW_DATA);

  rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(100), odom_timer_callback);
  rclc_timer_init_default(&motor_timer, &support, RCL_MS_TO_NS(20), motor_timer_callback);
  rclc_timer_init_default(&scan_timer, &support, RCL_MS_TO_NS(200), scan_timer_callback);

  rclc_executor_add_timer(&executor, &odom_timer);
  rclc_executor_add_timer(&executor, &motor_timer);
  rclc_executor_add_timer(&executor, &scan_timer);

  // 硬件初始化
  Wire.begin(I2C_SDA, I2C_SCL);
  // Wire.setClock(100000);  // 降低I2C速度为100kHz，解决通信错误
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pca9685.begin();
  pca9685.setPWMFreq(50);

  motor.attachMotor(0, MOTOR0_A_GPIO, MOTOR0_B_GPIO);
  motor.attachMotor(1, MOTOR1_A_GPIO, MOTOR1_B_GPIO);
  encoders[0].init(0, ENCODER0_A_GPIO, ENCODER0_B_GPIO);
  encoders[1].init(1, ENCODER1_A_GPIO, ENCODER1_B_GPIO);

  pidController[0].update_pid(PID_KP, PID_KI, PID_KD);
  pidController[1].update_pid(PID_KP, PID_KI, PID_KD);
  pidController[0].out_limit(-PID_OUT_LIMIT, PID_OUT_LIMIT);
  pidController[1].out_limit(-PID_OUT_LIMIT, PID_OUT_LIMIT);

  kinematics.set_motor_param(0, 1, ENCODER_PPR, WHEEL_DIAMETER);
  kinematics.set_motor_param(1, 1, ENCODER_PPR, WHEEL_DIAMETER);
  kinematics.set_kinematic_param(WHEEL_BASE);


  // 新增：手动初始化编码器和时间戳，避免首次dt超大
  motor_param_t& motor0 = ((Kinematics*)&kinematics)->motor_param_[0]; // 注意：需将Kinematics的motor_param_改为public，或新增初始化接口
  motor_param_t& motor1 = ((Kinematics*)&kinematics)->motor_param_[1];
  motor0.last_encoder_tick = encoders[0].getTicks();
  motor1.last_encoder_tick = encoders[1].getTicks();
  motor0.last_update_time = micros();
  motor1.last_update_time = micros();

  // ==================== VL53L0X 初始化（修复版！）====================
  // Wire.begin(I2C_SDA, I2C_SCL);
  // 错误的旧函数：vl53.init()
  // if (!vl53.begin()) {  // ✅ 正确：begin()
  //   Serial.println("VL53L0X 初始化失败");
  //   while (1);
  // }
  delay(100);  // 上电稳定延时
  // ==================== 终极修复：VL53L0X 初始化 ====================
  // 1. 先把 PCA9685 关掉（它是 I2C 干扰源！）
  pca9685.sleep();  // 🔥 休眠舵机驱动，消除干扰
  delay(50);

  // 2. 降低 I2C 速度
  Wire.setClock(50000);  // 超低速度，抗干扰拉满

  // 3. 强制初始化 VL53L0X
  bool vl_ok = false;
  for (int i = 0; i < 10; i++) {
    if (vl53.begin(0x29, &Wire)) {
      vl_ok = true;
      break;
    }
    delay(50);
  }

  if (!vl_ok) {
    Serial.println("VL53L0X 初始化失败 ❌");
    while (1);
  }

  // 4. 初始化完成后，恢复 I2C 速度
  Wire.setClock(100000);

  // 5. 唤醒舵机驱动
  pca9685.wakeup();
  delay(50);

  // 6. 配置传感器
  vl53.setMeasurementTimingBudgetMicroSeconds(20000);
  Serial.println("VL53L0X 初始化成功 ✅");




  ros_connected = true;
}


// void old_loop() {

  //   static unsigned long last_sync_time = 0;  
  // // 🔥 每 10秒 重新同步一次时间（防止漂移）
  // if (millis() - last_sync_time > 10000) {
  //   // 非阻塞尝试同步，失败也没关系，下次再来
  //   rmw_ret_t ret = rmw_uros_sync_session(100); 
  //   if (ret == RMW_RET_OK) {
  //     last_sync_time = millis();
  //   }
  // }

  // 1. 先扫一整圈数据，放到 capture_buffer 里
//   for (int i = 0; i < NUM_POINTS; i++) {
//     capture_buffer[i] = get_distance(i)+4.5;  
//   }

//   // 2. 扫完了，通知发布线程：“数据准备好了，你来拿”
//   new_data_ready = true;

//   // 注意：这里不需要 delay，扫完立刻开始扫下一圈
//   // micro-ROS 的循环处理也要记得加
//   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
// }

void loop() {
  static unsigned long last_sync_time = 0;
  if (millis() - last_sync_time > 10000) {
    rmw_uros_sync_session(100);
    last_sync_time = millis();
  }

  // ==========================
  // 正向扫描 0 → 120
  // ==========================
  for (int angle = 0; angle <= 120; angle++) {
    set_servo_angle(angle);          // 舵机转到当前角度
    delayMicroseconds(250);          // 最小必要延时（全速）
    capture_buffer[angle] = read_ultrasonic();  // 真实测距
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }
  new_data_ready = true;  // 一帧完成，交给ROS发布

  // ==========================
  // 反向扫描 120 → 0
  // ==========================
  for (int angle = 120; angle >= 0; angle--) {
    set_servo_angle(angle);
    delayMicroseconds(250);
    capture_buffer[angle] = read_ultrasonic();
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }
  new_data_ready = true;
}



