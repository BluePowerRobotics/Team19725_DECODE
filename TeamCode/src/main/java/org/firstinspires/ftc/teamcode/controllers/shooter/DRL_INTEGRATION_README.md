# DRL 模型集成文档

## 概述

本文档详细说明如何将 DRL 模型 `agent_DDPG.ser` 集成到发射速度运算中，控制炮台成 45 度，使用模型计算合适初速度后用 PID 发射小球。

## 模型输入输出格式

### 输入格式 (4 维向量)
- `input[0]`: 机器人速度 x 分量 (m/s)
- `input[1]`: 机器人速度 y 分量 (m/s)
- `input[2]`: 目标相对于机器人的 x 坐标 (cm)
- `input[3]`: 目标相对于机器人的 y 坐标 (cm)

### 输出格式 (3 维向量)
- `output[0]`: 发射初速度 x 分量 (cm/s)
- `output[1]`: 发射初速度 y 分量 (cm/s)
- `output[2]`: 发射初速度 z 分量 (cm/s)

## 代码结构

### 核心文件

1. **Shooter.java**
   - 单个弹射飞轮的 PID 控制器
   - DRL 模型加载和预测
   - 速度计算和控制逻辑

2. **ShooterAction.java**
   - 双发射轮协调控制
   - DRL 模式管理
   - 自动化发射动作

3. **DRLShooterTest.java**
   - 测试 OpMode
   - 用户交互界面
   - 实时数据显示

4. **TurretAngleController.java**
   - 炮台角度控制
   - 45 度角度设置
   - 舵机/电机控制

## 使用方法

### 1. 准备模型文件

将 `agent_DDPG.ser` 复制到机器人 SD 卡：
```
/sdcard/FIRST/agent_DDPG.ser
```

### 2. 配置硬件

在 FTC Robot Controller 中配置：
- `shooterMotor1`: 左发射轮
- `shooterMotor2`: 右发射轮
- `turretServo`: 炮台舵机
- `sweeperMotor`: 扫描器电机

### 3. 运行测试程序

1. 选择 `DRLShooterTest` OpMode
2. 点击 INIT 初始化
3. 点击 START 运行
4. 使用游戏手柄控制

### 4. 控制说明

| 按钮 | 功能 |
|------|------|
| A | 切换 DRL 模式 |
| B | 设置炮台角度为 45 度 |
| X | 更新机器人位置和速度 |
| Y | 发射小球 |
| DPad Up | 设置炮台角度为 60 度 |
| DPad Down | 设置炮台角度为 30 度 |
| DPad Left/Right | 设置炮台角度为 45 度 |
| Left Bumper | 扫描器输入 |
| Right Bumper | 扫描器输出 |
| Left Stick | 底盘控制 |

### 5. 集成到自动程序

```java
// 初始化系统
ShooterAction shooter = new ShooterAction(hardwareMap, telemetry);
TurretAngleController turretController = new TurretAngleController(hardwareMap, telemetry, "turretServo", true);
ChassisController chassis = new ChassisController(hardwareMap);
Sweeper_PID sweeper = new Sweeper_PID(hardwareMap, telemetry, "sweeperMotor", false);
Trigger trigger = new Trigger(hardwareMap);

// 设置炮台角度为 45 度
turretController.setTo45Degrees();

// 启用 DRL 模式
shooter.setUseDRL(true);

// 更新位置和速度信息
double robotX = chassis.robotPosition.getData().getPose2d().position.x;
double robotY = chassis.robotPosition.getData().getPose2d().position.y;
double targetX = 72; // 目标绝对位置
double targetY = 0;

// 计算目标相对位置
double targetRelX = targetX - robotX;
double targetRelY = targetY - robotY;

// 获取机器人速度（需要根据实际情况实现）
double robotVx = 0; // 例如：chassis.getVelocityX();
double robotVy = 0; // 例如：chassis.getVelocityY();

// 更新 ShooterAction 中的参数
shooter.robotVx = robotVx;
shooter.robotVy = robotVy;
shooter.targetRelX = targetRelX;
shooter.targetRelY = targetRelY;

// 使用 DRL 计算发射速度并射击
Actions.runBlocking(new SequentialAction(
    shooter.SpeedUpWithDRL(ShooterAction.targetSpeed_high),
    shooter.ShootThreeArtifactsWithDRL(ShooterAction.targetSpeed_high)
));
```

## 关键参数

### DRL 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| useDRL | boolean | false | 是否使用 DRL 模型 |
| robotVx | double | 0 | 机器人速度 x 分量 |
| robotVy | double | 0 | 机器人速度 y 分量 |
| targetRelX | double | 72 | 目标相对于机器人的 x 坐标 |
| targetRelY | double | 0 | 目标相对于机器人的 y 坐标 |
| turretAngle | double | π/4 | 炮台角度（45 度） |

### 炮台控制参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| servo45Position | double | 0.5 | 舵机 45 度位置 |
| angleP | double | 0.1 | PID 比例系数 |
| angleI | double | 0.01 | PID 积分系数 |
| angleD | double | 0.05 | PID 微分系数 |
| angleTolerance | double | 0.05 | 角度容差 |

### 发射轮参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| targetSpeed_low | int | 665 | 低目标速度 |
| targetSpeed_high | int | 835 | 高目标速度 |
| SpeedTolerance | int | 35 | 速度容差 |

## 故障排除

### 常见问题

1. **DRL 模型加载失败**
   - 检查模型文件路径是否正确
   - 确认 SD 卡已正确插入
   - 检查模型文件是否损坏

2. **发射精度问题**
   - 确保炮台角度正确设置为 45 度
   - 检查机器人速度获取是否准确
   - 调整 PID 参数以提高速度控制精度

3. **电机速度不稳定**
   - 检查电机连接是否松动
   - 调整 PID 参数
   - 确保电池电压充足

### 调试技巧

1. **使用 Dashboard 查看实时数据**
   - 打开 FTC Dashboard
   - 查看机器人位置、速度、目标位置等数据
   - 监控 DRL 模型的输入输出

2. **调整参数**
   - 根据实际测试结果调整 PID 参数
   - 调整 DRL 模型的输入输出范围
   - 优化炮台角度控制

3. **记录数据**
   - 记录成功和失败的发射数据
   - 分析数据以优化模型和参数

## 性能优化

1. **模型优化**
   - 使用更多训练数据训练模型
   - 调整模型结构以提高预测精度
   - 考虑使用在线学习方法

2. **硬件优化**
   - 使用更精确的传感器获取机器人速度
   - 优化发射轮和炮台的机械结构
   - 确保电源稳定

3. **算法优化**
   - 实现更精确的速度估计算法
   - 优化 PID 控制算法
   - 考虑使用卡尔曼滤波器融合传感器数据

## 总结

通过集成 DRL 模型，我们可以根据机器人速度和目标位置自动计算最优发射速度，提高发射精度和效率。同时，炮台 45 度控制确保了发射角度的一致性，进一步提高了射击精度。

使用本文档提供的代码和配置，您可以快速将 DRL 模型集成到您的机器人程序中，提升比赛表现。