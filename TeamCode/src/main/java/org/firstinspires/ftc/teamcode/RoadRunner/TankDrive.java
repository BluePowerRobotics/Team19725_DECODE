package org.firstinspires.ftc.teamcode.RoadRunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.messages.TankCommandMessage;
import org.firstinspires.ftc.teamcode.messages.TankLocalizerInputsMessage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

/**
 * 坦克驱动控制器，用于控制坦克驱动式机器人的运动和定位。
 * 
 * 此类实现了轨迹跟踪、转弯控制、姿态估计等功能，
 * 使用编码器进行定位，并支持轨迹规划和执行。
 */
@Config
public final class TankDrive {
    /**
     * 坦克驱动参数配置类，包含所有驱动和控制相关的参数。
     * 
     * 这些参数可以通过Dashboard进行实时调整，以优化机器人的运动性能。
     */
    public static class Params {
        /**
         * IMU传感器的Logo朝向，用于正确读取IMU数据。
         * 参考文档：https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
         */
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        
        /**
         * IMU传感器的USB接口朝向，用于正确读取IMU数据。
         */
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        /**
         * 每个编码器tick对应的英寸数，用于将编码器读数转换为实际距离。
         */
        public double inPerTick = 0;
        
        /**
         * 轮距，即左右轮之间的距离（以tick为单位）。
         */
        public double trackWidthTicks = 0;

        /**
         * 静态摩擦力补偿系数，用于电机前馈控制。
         */
        public double kS = 0;
        
        /**
         * 速度增益系数，用于电机前馈控制。
         */
        public double kV = 0;
        
        /**
         * 加速度增益系数，用于电机前馈控制。
         */
        public double kA = 0;

        /**
         * 最大轮速（英寸/秒），用于路径规划。
         */
        public double maxWheelVel = 50;
        
        /**
         * 最小轮廓加速度（英寸/秒²），用于路径规划。
         */
        public double minProfileAccel = -30;
        
        /**
         * 最大轮廓加速度（英寸/秒²），用于路径规划。
         */
        public double maxProfileAccel = 50;

        /**
         * 最大角速度（弧度/秒），用于转弯和路径规划。
         */
        public double maxAngVel = Math.PI; // shared with path
        
        /**
         * 最大角加速度（弧度/秒²），用于转弯规划。
         */
        public double maxAngAccel = Math.PI;

        /**
         * Ramsete控制器参数，范围(0, 1)，用于轨迹跟踪。
         */
        public double ramseteZeta = 0.7; // in the range (0, 1)
        
        /**
         * Ramsete控制器参数，正数，用于轨迹跟踪。
         */
        public double ramseteBBar = 2.0; // positive

        /**
         * 转弯控制器比例增益，用于转弯控制。
         */
        public double turnGain = 0.0;
        
        /**
         * 转弯控制器速度增益，用于转弯控制。
         */
        public double turnVelGain = 0.0;
    }

    /**
     * 全局参数实例，可通过Dashboard实时调整。
     */
    public static Params PARAMS = new Params();

    /**
     * 坦克驱动运动学模型，用于计算轮速和机器人运动之间的关系。
     */
    public final TankKinematics kinematics = new TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks);

    /**
     * 默认转弯约束，定义了最大角速度和角加速度。
     */
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    
    /**
     * 默认速度约束，结合了轮速限制和角速度限制。
     */
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    
    /**
     * 默认加速度约束，定义了最小和最大加速度。
     */
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    /**
     * 左侧电机列表，用于驱动左侧车轮。
     */
    public final List<DcMotorEx> leftMotors;
    
    /**
     * 右侧电机列表，用于驱动右侧车轮。
     */
    public final List<DcMotorEx> rightMotors;

    /**
     * 延迟初始化的IMU传感器，用于获取机器人的航向信息。
     */
    public final LazyImu lazyImu;

    /**
     * 电压传感器，用于读取电池电压，进行电压补偿。
     */
    public final VoltageSensor voltageSensor;

    /**
     * 定位器实例，用于估计机器人的位置和姿态。
     */
    public final Localizer localizer;
    
    /**
     * 姿态历史记录，用于绘制机器人的运动轨迹。
     */
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    /**
     * 估计姿态记录器，用于记录机器人的估计位置。
     */
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    
    /**
     * 目标姿态记录器，用于记录轨迹跟踪的目标位置。
     */
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    
    /**
     * 驱动命令记录器，用于记录发送给电机的驱动命令。
     */
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);

    /**
     * 坦克驱动命令记录器，用于记录左右轮的具体命令。
     */
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    /**
     * 驱动定位器类，实现了Localizer接口，使用编码器数据进行定位。
     * 
     * 此类通过读取左右轮编码器的读数，计算机器人的位移和旋转，
     * 从而估计机器人在场上的位置和姿态。
     */
    public class DriveLocalizer implements Localizer {
        /**
         * 左侧编码器列表，用于读取左侧车轮的旋转数据。
         */
        public final List<Encoder> leftEncs;
        
        /**
         * 右侧编码器列表，用于读取右侧车轮的旋转数据。
         */
        public final List<Encoder> rightEncs;
        
        /**
         * 当前机器人的姿态估计。
         */
        private Pose2d pose;

        /**
         * 上一次左侧编码器的位置读数。
         */
        private double lastLeftPos;
        
        /**
         * 上一次右侧编码器的位置读数。
         */
        private double lastRightPos;
        
        /**
         * 初始化标志，用于标记定位器是否已初始化。
         */
        private boolean initialized;

        /**
         * 构造函数，初始化驱动定位器。
         * 
         * @param pose 初始姿态估计，用于设置定位器的起始位置
         */
        public DriveLocalizer(Pose2d pose) {
            {
                // 初始化左侧编码器列表
                List<Encoder> leftEncs = new ArrayList<>();
                for (DcMotorEx m : leftMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    leftEncs.add(e);
                }
                this.leftEncs = Collections.unmodifiableList(leftEncs);
            }

            {
                // 初始化右侧编码器列表
                List<Encoder> rightEncs = new ArrayList<>();
                for (DcMotorEx m : rightMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    rightEncs.add(e);
                }
                this.rightEncs = Collections.unmodifiableList(rightEncs);
            }

            // TODO: reverse encoder directions if needed
            //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        /**
         * 设置定位器的当前姿态。
         * 
         * @param pose 要设置的目标姿态，包含x、y坐标和heading角度
         */
        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        /**
         * 获取当前估计的机器人姿态。
         * 
         * @return 当前机器人的姿态估计
         */
        @Override
        public Pose2d getPose() {
            return pose;
        }

        /**
         * 更新机器人的姿态估计。
         * 
         * 此方法通过读取编码器数据，计算机器人的位移和旋转，
         * 从而更新机器人的姿态估计。
         * 
         * @return 机器人的当前速度估计
         */
        @Override
        public PoseVelocity2d update() {
            // 临时变量，未使用
            Twist2dDual<Time> delta;

            // 存储编码器读数
            List<PositionVelocityPair> leftReadings = new ArrayList<>(), rightReadings = new ArrayList<>();
            double meanLeftPos = 0.0, meanLeftVel = 0.0;
            
            // 读取左侧编码器数据并计算平均值
            for (Encoder e : leftEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanLeftPos += p.position;
                meanLeftVel += p.velocity;
                leftReadings.add(p);
            }
            meanLeftPos /= leftEncs.size();
            meanLeftVel /= leftEncs.size();

            double meanRightPos = 0.0, meanRightVel = 0.0;
            
            // 读取右侧编码器数据并计算平均值
            for (Encoder e : rightEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanRightPos += p.position;
                meanRightVel += p.velocity;
                rightReadings.add(p);
            }
            meanRightPos /= rightEncs.size();
            meanRightVel /= rightEncs.size();

            // 记录编码器输入数据
            FlightRecorder.write("TANK_LOCALIZER_INPUTS",
                     new TankLocalizerInputsMessage(leftReadings, rightReadings));

            // 初始化处理
            if (!initialized) {
                initialized = true;

                lastLeftPos = meanLeftPos;
                lastRightPos = meanRightPos;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            // 计算轮速增量并转换为机器人运动
            Twist2dDual<Time> twist = kinematics.forward(new TankKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            meanLeftPos - lastLeftPos,  // 左侧位移
                            meanLeftVel                  // 左侧速度
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            meanRightPos - lastRightPos, // 右侧位移
                            meanRightVel,                // 右侧速度
                    }).times(PARAMS.inPerTick)
            ));

            // 更新编码器位置
            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            // 更新姿态估计
            pose = pose.plus(twist.value());

            // 返回速度估计
            return twist.velocity().value();
        }

        /**
         * 获取AprilTag视觉定位状态。
         * 
         * 由于此定位器仅使用编码器数据，不使用视觉定位，因此始终返回false。
         * 
         * @return false，因为此定位器不使用AprilTag
         */
        @Override
        public boolean getAprilTagStatus() {
            return false;
        }
    }

    /**
     * 构造函数，初始化坦克驱动控制器。
     * 
     * @param hardwareMap 硬件映射，用于获取硬件设备
     * @param pose 初始姿态估计，用于设置定位器的起始位置
     */
    public TankDrive(HardwareMap hardwareMap, Pose2d pose) {
        // 检查Lynx模块固件是否过时
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // 设置所有Lynx模块的缓存模式为自动
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // 初始化左侧电机列表
        // TODO: make sure your config has motors with these names (or change them)
        //   add additional motors on each side if you have them
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right"));

        // 设置所有电机的零功率行为为制动
        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse motor directions if needed
        //   leftMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        // 初始化IMU传感器
        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        // 获取电压传感器
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 初始化驱动定位器
        localizer = new DriveLocalizer(pose);

        // 记录坦克驱动参数
        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    /**
     * 设置驱动电机的功率，基于给定的机器人速度指令。
     * 
     * 此方法将机器人速度指令转换为左右轮的速度，
     * 并进行归一化处理以确保功率不超过最大值。
     * 
     * @param powers 机器人速度指令，包含线速度和角速度
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        // 将机器人速度指令转换为轮速
        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        // 计算最大功率幅度，用于归一化
        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        // 设置左侧电机功率
        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        // 设置右侧电机功率
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    /**
     * 轨迹跟踪动作类，实现了Action接口，用于执行轨迹跟踪。
     * 
     * 此类负责按照预定义的轨迹控制机器人运动，
     * 使用Ramsete控制器进行轨迹跟踪，并处理电机功率计算。
     */
    public final class FollowTrajectoryAction implements Action {
        /**
         * 时间轨迹，包含了轨迹的路径和速度轮廓。
         */
        public final TimeTrajectory timeTrajectory;
        
        /**
         * 动作开始时间戳，用于计算轨迹执行时间。
         */
        private double beginTs = -1;

        /**
         * 轨迹的x坐标点，用于绘制轨迹。
         */
        private final double[] xPoints;
        
        /**
         * 轨迹的y坐标点，用于绘制轨迹。
         */
        private final double[] yPoints;

        /**
         * 构造函数，初始化轨迹跟踪动作。
         * 
         * @param t 时间轨迹，包含了轨迹的路径和速度轮廓
         */
        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            // 生成轨迹的点集，用于绘制轨迹
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        /**
         * 执行轨迹跟踪动作，返回动作是否完成。
         * 
         * 此方法按照以下步骤执行轨迹跟踪：
         * 1. 计算当前轨迹执行时间
         * 2. 检查轨迹是否完成
         * 3. 获取当前时间的轨迹目标点
         * 4. 更新机器人姿态估计
         * 5. 使用Ramsete控制器计算控制命令
         * 6. 计算电机功率并设置
         * 7. 记录遥测数据并绘制轨迹
         * 
         * @param p 遥测数据包，用于记录和显示数据
         * @return true表示动作仍在执行，false表示动作已完成
         */
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            // 计算当前轨迹执行时间
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // 检查轨迹是否完成
            if (t >= timeTrajectory.duration) {
                // 停止所有电机
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false; // 动作完成
            }

            // 获取当前时间的轨迹进度
            DualNum<Time> x = timeTrajectory.profile.get(t);

            // 获取当前时间的轨迹目标点
            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            // 更新机器人姿态估计
            updatePoseEstimate();

            // 使用Ramsete控制器计算控制命令
            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, localizer.getPose());
            driveCommandWriter.write(new DriveCommandMessage(command));

            // 计算轮速并转换为电机功率
            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            // 设置电机功率
            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            // 记录遥测数据
            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            // 计算轨迹跟踪误差
            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // 绘制轨迹和机器人位置
            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            // 绘制目标机器人位置
            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            // 绘制当前机器人位置
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            // 绘制轨迹
            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true; // 动作仍在执行
        }

        /**
         * 预览轨迹，用于在执行前显示轨迹路径。
         * 
         * @param c 画布，用于绘制轨迹路径
         */
        @Override
        public void preview(Canvas c) {
            // 设置绘制参数
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            // 绘制轨迹路径
            c.strokePolyline(xPoints, yPoints);
        }
    }

    /**
     * 转弯动作类，实现了Action接口，用于执行转弯动作。
     * 
     * 此类负责按照预定义的转弯参数控制机器人旋转，
     * 使用比例控制器进行转弯控制，并处理电机功率计算。
     */
    public final class TurnAction implements Action {
        /**
         * 时间转弯，包含了转弯的角度和速度轮廓。
         */
        private final TimeTurn turn;

        /**
         * 动作开始时间戳，用于计算转弯执行时间。
         */
        private double beginTs = -1;

        /**
         * 构造函数，初始化转弯动作。
         * 
         * @param turn 时间转弯，包含了转弯的角度和速度轮廓
         */
        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        /**
         * 执行转弯动作，返回动作是否完成。
         * 
         * 此方法按照以下步骤执行转弯：
         * 1. 计算当前转弯执行时间
         * 2. 检查转弯是否完成
         * 3. 获取当前时间的转弯目标点
         * 4. 更新机器人姿态估计
         * 5. 使用比例控制器计算控制命令
         * 6. 计算电机功率并设置
         * 7. 记录遥测数据并绘制转弯轨迹
         * 
         * @param p 遥测数据包，用于记录和显示数据
         * @return true表示动作仍在执行，false表示动作已完成
         */
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            // 计算当前转弯执行时间
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // 检查转弯是否完成
            if (t >= turn.duration) {
                // 停止所有电机
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false; // 动作完成
            }

            // 获取当前时间的转弯目标点
            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            // 更新机器人姿态估计
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            // 使用比例控制器计算控制命令
            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),  // 线速度为0
                    txWorldTarget.heading.velocity().plus(
                            // 比例控制项：角度误差
                            PARAMS.turnGain * localizer.getPose().heading.minus(txWorldTarget.heading.value()) +
                            // 速度控制项：角速度误差
                            PARAMS.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                    )
            );
            driveCommandWriter.write(new DriveCommandMessage(command));

            // 计算轮速并转换为电机功率
            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            // 设置电机功率
            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            // 绘制转弯轨迹和机器人位置
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            // 绘制目标机器人位置
            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            // 绘制当前机器人位置
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            // 绘制转弯中心点
            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true; // 动作仍在执行
        }

        /**
         * 预览转弯，用于在执行前显示转弯中心点。
         * 
         * @param c 画布，用于绘制转弯中心点
         */
        @Override
        public void preview(Canvas c) {
            // 设置绘制参数
            c.setStroke("#7C4DFF7A");
            // 绘制转弯中心点
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    /**
     * 更新机器人的姿态估计，并记录姿态历史。
     * 
     * 此方法调用定位器的update方法获取最新的姿态估计，
     * 并将其添加到姿态历史记录中，用于绘制机器人的运动轨迹。
     * 
     * @return 机器人的当前速度估计
     */
    public PoseVelocity2d updatePoseEstimate() {
        // 调用定位器更新姿态估计
        PoseVelocity2d vel = localizer.update();
        // 添加到姿态历史记录
        poseHistory.add(localizer.getPose());

        // 保持姿态历史记录的大小不超过100
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        // 记录估计姿态
        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));

        return vel;
    }

    /**
     * 绘制姿态历史，用于显示机器人的运动轨迹。
     * 
     * 此方法将姿态历史记录中的位置点转换为坐标数组，
     * 并使用画布绘制出机器人的运动轨迹。
     * 
     * @param c 画布，用于绘制运动轨迹
     */
    private void drawPoseHistory(Canvas c) {
        // 准备坐标数组
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        // 填充坐标数组
        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        // 设置绘制参数并绘制轨迹
        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    /**
     * 创建轨迹动作构建器，用于构建复杂的轨迹动作序列。
     * 
     * 此方法返回一个TrajectoryActionBuilder实例，可用于构建包含
     * 直线移动、转弯等动作的轨迹序列。
     * 
     * @param beginPose 轨迹的起始姿态
     * @return 轨迹动作构建器实例
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,                    // 转弯动作工厂
                FollowTrajectoryAction::new,         // 轨迹跟踪动作工厂
                new TrajectoryBuilderParams(
                        1e-6,                          // 路径构建的精度
                        new ProfileParams(
                                0.25, 0.1, 1e-2        // 路径规划参数
                        )
                ),
                beginPose, 0.0,                      // 起始姿态和初始方向
                defaultTurnConstraints,             // 默认转弯约束
                defaultVelConstraint, defaultAccelConstraint  // 默认速度和加速度约束
        );
    }
}
