package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

/**
 * 双轮定位器实现，使用两个编码器和IMU来估计机器人的位置和姿态。
 * 
 * 此定位器使用一个平行于机器人前进方向的编码器（par）和一个垂直于前进方向的编码器（perp），
 * 结合IMU提供的航向信息，来计算机器人的位置和姿态。
 */
@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    /**
     * 定位器参数类，用于存储编码器安装位置的参数。
     * 这些参数用于补偿机器人旋转时编码器的位置偏移。
     */
    public static class Params {
        /**
         * 平行编码器的Y轴位置（以tick为单位）。
         * 表示平行编码器相对于机器人旋转中心的Y轴偏移量。
         */
        public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        
        /**
         * 垂直编码器的X轴位置（以tick为单位）。
         * 表示垂直编码器相对于机器人旋转中心的X轴偏移量。
         */
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    /**
     * 定位器参数实例，可通过Dashboard进行调整。
     */
    public static Params PARAMS = new Params();

    /**
     * 平行于机器人前进方向的编码器（测量前进/后退距离）。
     */
    public final Encoder par;
    
    /**
     * 垂直于机器人前进方向的编码器（测量横向移动距离）。
     */
    public final Encoder perp;
    
    /**
     * IMU传感器，用于获取机器人的航向和角速度。
     */
    public final IMU imu;

    /**
     * 上次平行编码器的位置（用于计算位置变化）。
     */
    private int lastParPos;
    
    /**
     * 上次垂直编码器的位置（用于计算位置变化）。
     */
    private int lastPerpPos;
    
    /**
     * 上次的航向角度（用于计算角度变化）。
     */
    private Rotation2d lastHeading;

    /**
     * 每个编码器tick对应的英寸数，用于将编码器读数转换为实际距离。
     */
    private final double inPerTick;

    /**
     * 上次的原始航向角速度（用于处理角度溢出问题）。
     */
    private double lastRawHeadingVel;
    
    /**
     * 航向角速度偏移量（用于处理角度溢出问题）。
     */
    private double headingVelOffset;
    
    /**
     * 定位器是否已初始化。
     */
    private boolean initialized;
    
    /**
     * 当前机器人的姿态估计。
     */
    private Pose2d pose;

    /**
     * 构造函数，初始化双轮定位器。
     * 
     * @param hardwareMap 硬件映射，用于获取编码器和IMU
     * @param imu IMU传感器实例
     * @param inPerTick 每个编码器tick对应的英寸数
     * @param initialPose 初始姿态估计
     */
    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick, Pose2d initialPose) {
        // 初始化平行编码器
        // 注意：确保配置文件中有名称为"par"的电机，编码器应插入匹配的插槽
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
        
        // 初始化垂直编码器
        // 注意：确保配置文件中有名称为"perp"的电机，编码器应插入匹配的插槽
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: 如果需要，反转编码器方向
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = imu;
        this.inPerTick = inPerTick;

        // 将参数写入飞行记录器，用于调试
        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);

        // 设置初始姿态
        pose = initialPose;
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
     * 获取当前姿态估计。
     * 
     * @return 定位器当前的姿态估计，包含x、y坐标和heading角度
     */
    @Override
    public Pose2d getPose() {
        return pose;
    }

    /**
     * 更新定位器的姿态估计。
     * 此方法会根据编码器和IMU数据计算并更新机器人的当前姿态。
     * 
     * @return 定位器当前的速度估计，包含线速度和角速度
     */
    @Override
    public PoseVelocity2d update() {
        // 获取编码器的位置和速度
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        // 获取IMU的角度和角速度
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        // 使用度数来解决角度单位问题
        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        // 将角速度转换为弧度/秒
        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                angularVelocityDegrees.acquisitionTime
        );

        // 将输入数据写入飞行记录器，用于调试
        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        // 计算当前航向
        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        // 处理IMU角速度的溢出问题
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        // 初始化检查
        if (!initialized) {
            initialized = true;

            // 记录初始状态
            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            // 初始速度为零
            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        // 计算编码器位置变化和航向变化
        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        // 计算机器人的运动增量（考虑编码器位置偏移的影响）
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        // 计算前进/后退方向的运动（补偿旋转影响）
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        // 计算横向运动（补偿旋转影响）
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                // 计算旋转运动
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        // 更新状态变量
        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        // 更新姿态估计
        pose = pose.plus(twist.value());
        
        // 返回速度估计
        return twist.velocity().value();
    }

    /**
     * 获取AprilTag视觉定位的状态。
     * 由于双轮定位器不使用AprilTag，因此始终返回false。
     * 
     * @return 始终返回false
     */
    @Override
    public boolean getAprilTagStatus() {
        return false;
    }
}
