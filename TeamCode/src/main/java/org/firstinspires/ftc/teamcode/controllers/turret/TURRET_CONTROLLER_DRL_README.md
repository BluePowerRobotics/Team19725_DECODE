# TurretController DRL 集成文档

## 概述

本文档详细说明如何使用 DRL 模型重新实现 TurretController.java 的速度计算功能。新的实现支持使用 DRL 模型来计算发射速度，同时保留了传统的物理计算方法。

## 主要功能

### 1. DRL 模型集成
- 自动加载 DRL 模型 `agent_DDPG.ser`
- 支持实时切换 DRL 模式和传统模式
- 使用机器人速度和目标相对位置作为输入
- 输出 3D 发射速度向量

### 2. 速度计算方法
- `solveSpeedWithDRL()`: 使用 DRL 模型计算发射速度
- `solveSpeed()`: 传统物理计算方法（固定仰角）
- `solveTheta()`: 传统物理计算方法（固定初速度）

### 3. 灵活的控制接口
- 支持固定仰角计算发射速度
- 支持固定速度计算仰角
- 支持多种输入格式（坐标、速度、位置对象）

## 代码结构

### TurretController 类

```java
public class TurretController {
    private Agent drlAgent;              // DRL 模型
    private boolean drlModelLoaded;      // 模型加载状态
    private boolean useDRLModel;         // 是否使用 DRL 模型
    private Telemetry telemetry;         // 遥测接口
    
    // 构造函数
    public TurretController(Telemetry telemetry);
    public TurretController();
    
    // DRL 模型管理
    private void loadDRLModel();
    public void setUseDRLModel(boolean use);
    public boolean isDRLModelLoaded();
    public boolean isUseDRLModel();
    
    // 计算器创建
    public TurretCalculator createCalculator();
    public TurretCalculator createCalculator(boolean useDRL);
}
```

### TurretCalculator 类

```java
class TurretCalculator {
    private Agent drlAgent;              // DRL 模型
    private boolean useDRLModel;         // 是否使用 DRL 模型
    
    // 构造函数
    public TurretCalculator(Agent drlAgent, boolean useDRLModel);
    
    // DRL 模型设置
    public void setDRLAgent(Agent drlAgent);
    public void setUseDRLModel(boolean use);
    
    // DRL 速度计算
    public List<TurretInfo> solveSpeedWithDRL(double theta, double x, double y, double vx, double vy);
    public List<TurretInfo> solveSpeedWithDRL(double theta, Pose2d pose2d, PoseVelocity2d poseVelocity2d);
    
    // 传统速度计算
    public List<TurretInfo> solveSpeed(double theta, double x, double y, double vx, double vy);
    public List<TurretInfo> solveSpeed(double theta, Pose2d pose2d, PoseVelocity2d poseVelocity2d);
    
    // 角度计算
    public List<TurretInfo> solveTheta(double speed, double x, double y, double vx, double vy);
    public List<TurretInfo> solveTheta(double speed, Pose2d pose2d, PoseVelocity2d poseVelocity2d);
    
    // 参数类
    public static class Param {
        public static double g = 9.8;           // 重力加速度
        public static Point3D target;          // 目标位置
        public static double turretHeight;      // 炮台高度
    }
}
```

## 使用方法

### 1. 基本初始化

```java
// 创建 TurretController 实例
TurretController turretController = new TurretController(telemetry);

// 设置目标位置
TurretController.TurretCalculator.Param.target = new Point3D(2.0, 0.0, 0.5);
TurretController.TurretCalculator.Param.turretHeight = 0.1;

// 启用 DRL 模式
turretController.setUseDRLModel(true);
```

### 2. 计算发射速度（固定仰角）

```java
// 创建计算器
TurretController.TurretCalculator calculator = turretController.createCalculator();

// 使用 DRL 模型计算
List<TurretInfo> results = calculator.solveSpeedWithDRL(
    Math.PI / 4,  // 仰角 45 度
    0.0,          // 炮台 x 位置
    0.0,          // 炮台 y 位置
    0.5,          // 炮台 x 速度
    0.0           // 炮台 y 速度
);

// 处理结果
for (TurretInfo info : results) {
    telemetry.addData("Speed", info.v + " m/s");
    telemetry.addData("Phi", Math.toDegrees(info.phi) + "°");
    telemetry.addData("Theta", Math.toDegrees(info.theta) + "°");
    telemetry.addData("Time", info.t + " s");
}
```

### 3. 使用位置对象计算

```java
// 获取机器人位置和速度
Pose2d pose2d = chassis.robotPosition.getData().getPose2d();
PoseVelocity2d poseVelocity2d = new PoseVelocity2d(
    new Vector2d(0.5, 0.0),  // 线速度
    0.0                       // 角速度
);

// 使用 DRL 模型计算
List<TurretInfo> results = calculator.solveSpeedWithDRL(
    Math.PI / 4,    // 仰角
    pose2d,         // 位置
    poseVelocity2d  // 速度
);
```

### 4. 切换计算模式

```java
// 使用 DRL 模型
turretController.setUseDRLModel(true);
List<TurretInfo> drlResults = calculator.solveSpeedWithDRL(theta, x, y, vx, vy);

// 使用传统方法
turretController.setUseDRLModel(false);
List<TurretInfo> traditionalResults = calculator.solveSpeed(theta, x, y, vx, vy);
```

## DRL 模型输入输出

### 输入格式 (4 维向量)
- `input[0]`: 机器人速度 x 分量 (m/s)
- `input[1]`: 机器人速度 y 分量 (m/s)
- `input[2]`: 目标相对于机器人的 x 坐标 (m)
- `input[3]`: 目标相对于机器人的 y 坐标 (m)

### 输出格式 (3 维向量)
- `output[0]`: 发射初速度 x 分量 (m/s)
- `output[1]`: 发射初速度 y 分量 (m/s)
- `output[2]`: 发射初速度 z 分量 (m/s)

### 转换为 TurretInfo
```java
double v = Math.sqrt(action[0] * action[0] + action[1] * action[1] + action[2] * action[2]);
double phi = Math.atan2(action[1], action[0]);
```

## 测试程序

### TurretControllerTest OpMode

**控制说明**：
- **A 键**: 切换 DRL 模式
- **B 键**: 设置炮台角度为 45 度
- **X 键**: 更新目标位置
- **Y 键**: 计算并显示炮台信息
- **DPad Up/Down**: 调整仰角 theta

**参数配置**：
```java
public static boolean useDRL = false;
public static double targetX = 2.0;
public static double targetY = 0.0;
public static double targetZ = 0.5;
public static double turretHeight = 0.1;
public static double theta = Math.PI / 4;
```

## 集成示例

### 完整的自动程序集成

```java
// 初始化
TurretController turretController = new TurretController(telemetry);
TurretAngleController turretAngleController = new TurretAngleController(hardwareMap, telemetry, "turretServo", true);
ChassisController chassis = new ChassisController(hardwareMap);

// 设置目标
TurretController.TurretCalculator.Param.target = new Point3D(2.0, 0.0, 0.5);
TurretController.TurretCalculator.Param.turretHeight = 0.1;

// 启用 DRL 模式
turretController.setUseDRLModel(true);

// 设置炮台角度
turretAngleController.setTo45Degrees();

// 计算发射参数
TurretController.TurretCalculator calculator = turretController.createCalculator();
Pose2d pose2d = chassis.robotPosition.getData().getPose2d();
PoseVelocity2d poseVelocity2d = new PoseVelocity2d(
    new Vector2d(0.5, 0.0),
    0.0
);

List<TurretInfo> results = calculator.solveSpeedWithDRL(
    Math.PI / 4,    // 45 度仰角
    pose2d,
    poseVelocity2d
);

// 使用计算结果
if (!results.isEmpty()) {
    TurretInfo info = results.get(0);
    double launchSpeed = info.v;
    double launchAngle = Math.toDegrees(info.phi);
    
    // 设置发射速度
    shooter.setShootSpeed((int) (launchSpeed * 100));
    
    // 发射小球
    trigger.open();
    sweeper.SweeperAction(Sweeper_PID.GiveTheArtifactVel);
    sleep(1000);
    sweeper.Sweep(0);
    trigger.close();
}
```

## 参数说明

### 物理参数
- `g`: 重力加速度 (默认 9.8 m/s²)
- `target`: 目标位置 (Point3D)
- `turretHeight`: 炮台高度 (m)

### DRL 参数
- `useDRLModel`: 是否使用 DRL 模型
- `drlModelLoaded`: DRL 模型是否加载成功

### 计算参数
- `theta`: 炮台仰角 (rad)
- `phi`: 水平发射角 (rad)
- `v`: 发射初速度 (m/s)
- `t`: 飞行时间 (s)

## 故障排除

### 常见问题

1. **DRL 模型加载失败**
   - 检查模型文件路径是否正确
   - 确认 SD 卡已正确插入
   - 检查模型文件是否损坏

2. **计算结果不准确**
   - 确保目标位置设置正确
   - 检查机器人位置和速度获取是否准确
   - 调整 DRL 模型参数或重新训练

3. **炮台角度控制不稳定**
   - 检查 TurretAngleController 的 PID 参数
   - 确保炮台机械结构正常
   - 调整角度容差参数

### 调试技巧

1. **使用 Dashboard 查看实时数据**
   - 打开 FTC Dashboard
   - 查看计算结果和模型输出
   - 监控机器人状态

2. **对比 DRL 和传统方法**
   - 同时使用两种方法计算
   - 对比结果差异
   - 分析 DRL 模型的准确性

3. **记录和分析数据**
   - 记录成功和失败的计算案例
   - 分析输入输出关系
   - 优化模型和参数

## 性能优化

1. **模型优化**
   - 使用更多训练数据
   - 调整模型结构
   - 实现在线学习

2. **计算优化**
   - 缓存计算结果
   - 优化算法复杂度
   - 使用并行计算

3. **硬件优化**
   - 使用更精确的传感器
   - 优化机械结构
   - 确保电源稳定

## 总结

通过集成 DRL 模型，TurretController 现在支持智能发射速度计算，可以根据机器人速度和目标位置自动计算最优发射参数。同时保留了传统物理计算方法，确保系统的可靠性和灵活性。

使用本文档提供的代码和配置，您可以快速将 DRL 模型集成到炮台控制系统中，提升射击精度和效率。