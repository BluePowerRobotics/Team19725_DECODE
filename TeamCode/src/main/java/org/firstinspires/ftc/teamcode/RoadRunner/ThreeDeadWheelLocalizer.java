package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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

import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;

/**
 * 三轮定位器实现，使用三个编码器来估计机器人的位置和姿态。
 * 
 * 此定位器使用两个平行于机器人前进方向的编码器（par0, par1）和一个垂直于前进方向的编码器（perp），
 * 不需要IMU即可计算机器人的位置和姿态。通过两个平行编码器的读数差异，可以计算出机器人的旋转角度。
 */
@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    /**
     * 定位器参数类，用于存储编码器安装位置的参数。
     * 这些参数用于计算机器人的运动和旋转。
     */
    public static class Params {
        /**
         * 第一个平行编码器的Y轴位置（以tick为单位）。
         * 表示第一个平行编码器相对于机器人旋转中心的Y轴偏移量。
         */
        public double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
        
        /**
         * 第二个平行编码器的Y轴位置（以tick为单位）。
         * 表示第二个平行编码器相对于机器人旋转中心的Y轴偏移量。
         */
        public double par1YTicks = 1.0; // y position of the second parallel encoder (in tick units)
        
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
     * 第一个平行于机器人前进方向的编码器（测量前进/后退距离）。
     */
    public final Encoder par0;
    
    /**
     * 第二个平行于机器人前进方向的编码器（测量前进/后退距离，用于计算旋转）。
     */
    public final Encoder par1;
    
    /**
     * 垂直于机器人前进方向的编码器（测量横向移动距离）。
     */
    public final Encoder perp;

    /**
     * 每个编码器tick对应的英寸数，用于将编码器读数转换为实际距离。
     */
    public final double inPerTick;

    /**
     * 上次第一个平行编码器的位置（用于计算位置变化）。
     */
    private int lastPar0Pos;
    
    /**
     * 上次第二个平行编码器的位置（用于计算位置变化）。
     */
    private int lastPar1Pos;
    
    /**
     * 上次垂直编码器的位置（用于计算位置变化）。
     */
    private int lastPerpPos;
    
    /**
     * 定位器是否已初始化。
     */
    private boolean initialized;
    
    /**
     * 当前机器人的姿态估计。
     */
    private Pose2d pose;

    /**
     * 构造函数，初始化三轮定位器。
     * 
     * @param hardwareMap 硬件映射，用于获取编码器
     * @param inPerTick 每个编码器tick对应的英寸数
     * @param initialPose 初始姿态估计
     */
    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // 初始化第一个平行编码器
        // 注意：确保配置文件中有名称为"par0"的电机，编码器应插入匹配的插槽
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0")));
        
        // 初始化第二个平行编码器
        // 注意：确保配置文件中有名称为"par1"的电机，编码器应插入匹配的插槽
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
        
        // 初始化垂直编码器
        // 注意：确保配置文件中有名称为"perp"的电机，编码器应插入匹配的插槽
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: 如果需要，反转编码器方向
        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        // 将参数写入飞行记录器，用于调试
        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);

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
     * 此方法会根据三个编码器的数据计算并更新机器人的当前姿态。
     * 
     * @return 定位器当前的速度估计，包含线速度和角速度
     */
    @Override
    public PoseVelocity2d update() {
        // 获取三个编码器的位置和速度
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        // 将输入数据写入飞行记录器，用于调试
        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        // 初始化检查
        if (!initialized) {
            initialized = true;

            // 记录初始状态
            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            // 初始速度为零
            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        // 计算编码器位置变化
        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        // 计算机器人的运动增量
        // 使用两个平行编码器的差异来计算旋转，使用垂直编码器来计算横向运动
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        // 计算前进/后退方向的运动
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        // 计算横向运动（补偿旋转影响）
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                // 计算旋转运动（基于两个平行编码器的差异）
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        // 更新状态变量
        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        // 更新姿态估计
        pose = pose.plus(twist.value());
        
        // 返回速度估计
        return twist.velocity().value();
    }

    /**
     * 获取AprilTag视觉定位的状态。
     * 由于三轮定位器不使用AprilTag，因此始终返回false。
     * 
     * @return 始终返回false
     */
    @Override
    public boolean getAprilTagStatus() {
        return false;
    }
}
