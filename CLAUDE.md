# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

本项目是基于PX4专业级开源自动驾驶仪开源项目二次开发的固定翼飞控系统。硬件使用的是Pixhawk2.4.8。
## 构建系统详解

### 基础构建命令

```bash
# SITL（软件在环仿真）
make px4_sitl_default                  # 默认SITL构建
make px4_sitl_default gazebo-classic   # 使用Gazebo Classic仿真器
make px4_sitl_default gz               # 使用新版Gazebo Garden
make px4_sitl_default jmavsim          # 使用jMAVSim仿真器

# 常用硬件目标
make px4_fmu-v5_default                # Pixhawk 4
make px4_fmu-v6x_default               # Pixhawk 6X
make px4_fmu-v6c_default               # Pixhawk 6C
make holybro_kakuteh7v2_default        # Kakute H7 V2

# 特殊构建变体
make px4_fmu-v5_test                   # 包含测试的构建
make px4_fmu-v5_bootloader            # 引导程序构建

# 清理和重建
make clean                             # 清理构建
make distclean                         # 深度清理（包括子模块）
```

### 高级构建选项

```bash
# 并行构建（加速编译）
make -j8 px4_sitl_default

# 指定构建目录
PX4_CMAKE_BUILD_DIR=build_custom make px4_sitl_default

# 启用编译器优化
CMAKE_BUILD_TYPE=Release make px4_sitl_default

# 调试构建
CMAKE_BUILD_TYPE=Debug make px4_sitl_default
```

### 构建系统架构

- **Makefile包装器**: 根目录的Makefile提供简化接口
- **CMake核心**: 实际构建逻辑在CMakeLists.txt中
- **Kconfig系统**: 用于模块和功能配置
- **板级配置**: boards/目录下的.px4board文件定义具体配置

## 测试体系

### 单元测试

```bash
# 运行所有单元测试
make tests

# 运行特定模块测试
make tests TESTFILTER=commander
make tests TESTFILTER=ekf2

# 使用GTest的高级过滤
make tests GTEST_FILTER="EKF2.*:Commander.*"

# 生成测试覆盖率报告
make coverage
```

### 集成测试

```bash
# SITL集成测试
make tests_integration

# 特定测试场景
make tests_integration TESTFILTER=mission
make tests_integration TESTFILTER=multicopter

# MAVROS测试
make tests_integration_mavros
```

### 硬件在环测试（HIL）

```bash
# 准备HIL测试
Tools/HIL/run_tests.py --device /dev/ttyUSB0
```

## 核心架构深度解析

### uORB消息系统

uORB（微对象请求代理）是PX4的核心通信机制，基于高效的发布-订阅模式。

#### 消息定义和生成
```bash
# 消息定义位置
msg/                                   # 所有uORB消息定义（.msg文件）

# 关键消息类型
msg/VehicleStatus.msg                  # 飞行器状态
msg/VehicleAttitude.msg               # 姿态数据
msg/VehicleLocalPosition.msg          # 本地位置
msg/SensorCombined.msg                # 传感器融合数据
msg/ActuatorOutputs.msg               # 执行器输出
```

#### uORB使用模式
```cpp
// 发布消息
#include <uORB/topics/vehicle_status.h>
orb_advert_t _status_pub = orb_advertise(ORB_ID(vehicle_status), &status);
orb_publish(ORB_ID(vehicle_status), _status_pub, &status);

// 订阅消息
int _status_sub = orb_subscribe(ORB_ID(vehicle_status));
vehicle_status_s status;
orb_copy(ORB_ID(vehicle_status), _status_sub, &status);

// 检查更新
bool updated;
orb_check(_status_sub, &updated);
if (updated) {
    orb_copy(ORB_ID(vehicle_status), _status_sub, &status);
}
```

### 模块系统架构

#### 标准模块结构
```
src/modules/my_module/
├── CMakeLists.txt                    # CMake配置
├── Kconfig                           # 模块配置选项
├── my_module.cpp                     # 主实现文件
├── my_module.h                       # 头文件
├── module.yaml                       # 模块元数据
└── params.c                          # 参数定义
```

#### 模块生命周期
```cpp
class MyModule : public ModuleBase<MyModule>, public px4::ScheduledWorkItem
{
public:
    MyModule();
    ~MyModule() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void Run() override;  // 主循环

private:
    // 初始化
    bool init();
    
    // 参数更新
    void parameters_update();
    
    // uORB订阅
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    
    // uORB发布
    uORB::PublicationMulti<custom_msg_s> _custom_pub{ORB_ID(custom_msg)};
};
```

### 核心控制流程

```
传感器数据采集 → 传感器融合(sensors) → 状态估计(ekf2) → 
飞行模式管理(flight_mode_manager) → 位置/姿态控制 → 
控制分配(control_allocator) → 执行器输出
```

#### 关键模块职责

1. **commander**: 系统状态管理、故障检测、模式切换
2. **navigator**: 任务执行、航点管理、地理围栏
3. **ekf2**: 24状态扩展卡尔曼滤波器，提供位置、速度、姿态估计
4. **flight_mode_manager**: 飞行模式执行框架
5. **mc_pos_control/fw_pos_control**: 位置控制器
6. **mc_att_control/fw_att_control**: 姿态控制器
7. **control_allocator**: 控制分配，将控制指令映射到执行器

## 参数系统使用

### 参数定义
```cpp
// 在模块的params.c文件中
/**
 * 横滚P增益
 *
 * @min 0.0
 * @max 12.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 6.5f);
```

### 参数访问
```cpp
// 声明参数句柄
param_t _param_mc_roll_p;

// 初始化
_param_mc_roll_p = param_find("MC_ROLL_P");

// 获取参数值
float roll_p;
param_get(_param_mc_roll_p, &roll_p);

// 监听参数更新
if (_parameter_update_sub.updated()) {
    parameter_update_s update;
    _parameter_update_sub.copy(&update);
    // 重新加载参数
}
```

## 驱动开发指南

### 创建新传感器驱动

1. **目录结构**
```
src/drivers/imu/my_imu/
├── CMakeLists.txt
├── Kconfig
├── my_imu.cpp
├── my_imu.h
└── parameters.yaml
```

2. **驱动基类**
```cpp
class MyIMU : public I2CDevice, public px4::ScheduledWorkItem
{
public:
    MyIMU(const I2CSPIDriverConfig &config);
    
    static int instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator);
    
    void RunImpl();
    int init() override;
    void print_status() override;

private:
    PX4Accelerometer _px4_accel;
    PX4Gyroscope _px4_gyro;
};
```

3. **注册驱动**
```cpp
// 在驱动文件末尾
DECLARE_DRIVER(my_imu, MyIMU);
```

## 调试技巧

### SITL调试

```bash
# 启动SITL并附加GDB
make px4_sitl_default
gdb build/px4_sitl_default/bin/px4

# GDB常用命令
(gdb) break MyModule::Run
(gdb) continue
(gdb) backtrace
(gdb) print _vehicle_status
```

### 实时日志监控

```bash
# 使用MAVLink shell
Tools/mavlink_shell.py /dev/ttyUSB0

# 在NSH中查看日志
nsh> dmesg
nsh> top
nsh> listener vehicle_status
nsh> param show MC_ROLL_P
```

### 日志分析

```bash
# 下载日志
Tools/upload_log.py log_file.ulg

# 分析EKF性能
python Tools/ecl_ekf/process_logdata_ekf.py log_file.ulg

# 在线分析
# 上传到 https://review.px4.io
```

## 性能优化

### 内存管理最佳实践

1. **避免动态分配**
```cpp
// 错误示例
float *data = new float[100];  // 避免！

// 正确示例
static float data[100];         // 使用静态分配
```

2. **使用内存池**
```cpp
// 使用预分配的缓冲区
ORB_DECLARE_MULTI(vehicle_odometry, ORB_MULTI_MAX_INSTANCES);
```

### 实时性保证

1. **避免阻塞操作**
```cpp
// 使用非阻塞I/O
int ret = ::read(_fd, buffer, sizeof(buffer));
if (ret < 0 && errno == EAGAIN) {
    // 无数据可读，立即返回
    return;
}
```

2. **控制循环频率**
```cpp
// 设置模块运行频率
ScheduleOnInterval(20_ms);  // 50Hz
```

### CPU使用优化

```cpp
// 使用性能计数器
perf_counter_t _loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");

void Run() {
    perf_begin(_loop_perf);
    // 实际工作
    perf_end(_loop_perf);
}
```

## 常见开发场景

### 添加新飞行模式

1. **创建飞行任务**
```cpp
// src/modules/flight_mode_manager/tasks/MyMode/FlightTaskMyMode.hpp
class FlightTaskMyMode : public FlightTask
{
public:
    bool activate(const trajectory_setpoint_s &last_setpoint) override;
    bool update() override;

private:
    void _generateSetpoints();
};
```

2. **注册到飞行模式管理器**
```cpp
// 在FlightModeManager中添加
case FlightMode::MyMode:
    task = new FlightTaskMyMode();
    break;
```

### 自定义MAVLink消息处理

```cpp
// 在mavlink模块中添加处理器
void handle_message_command_long(mavlink_message_t *msg)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);
    
    if (cmd.command == MAV_CMD_MY_CUSTOM) {
        // 处理自定义命令
    }
}
```

### 实现新的控制算法

1. **位置控制器扩展**
```cpp
// 继承并重写控制方法
class MyPositionControl : public PositionControl
{
protected:
    void _positionControl() override;
    void _velocityControl() override;
};
```

2. **添加新的控制模式**
```cpp
// 在控制器中添加新模式处理
switch (_control_mode) {
case ControlMode::MyMode:
    _myModeControl();
    break;
}
```

## 工具使用说明

### 参数生成和文档

```bash
# 生成参数元数据
make parameters_metadata

# 生成模块文档
make module_documentation
```

### 代码格式化

```bash
# 检查格式
make check_format

# 自动格式化
make format

# 仅格式化修改的文件
Tools/astyle/check_code_style.sh --fix
```

### 静态分析

```bash
# Clang-tidy分析
make clang-tidy

# CppCheck分析
make cppcheck

# 扫描编译警告
make scan-build
```

## 最佳实践总结

1. **始终遵循PX4编码规范**：使用Linux内核风格，tab缩进，最大行长120字符
2. **优先考虑实时性**：避免动态内存分配和阻塞操作
3. **充分利用uORB**：松耦合设计，清晰的数据流
4. **编写可测试代码**：模块化设计，便于单元测试
5. **注重性能监控**：使用perf计数器跟踪关键路径
6. **详细的参数文档**：为所有参数提供清晰的描述和范围
7. **遵循git工作流**：功能分支开发，PR审查流程

## 故障排除

### 常见构建问题

```bash
# 子模块问题
git submodule update --init --recursive

# 清理并重建
make distclean
make px4_sitl_default

# 工具链问题
Tools/setup/ubuntu.sh  # Ubuntu环境设置
```

### 仿真问题

```bash
# Gazebo启动失败
killall gzserver gzclient
make px4_sitl_default gazebo-classic

# MAVLink连接问题
# 检查 QGroundControl 是否占用端口
```

### 硬件调试

```bash
# 串口权限
sudo usermod -a -G dialout $USER
# 注销并重新登录

# 固件上传失败
# 尝试手动进入bootloader模式
```