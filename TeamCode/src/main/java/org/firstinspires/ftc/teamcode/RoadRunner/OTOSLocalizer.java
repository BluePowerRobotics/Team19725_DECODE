package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.OTOSKt;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * SparkFun OTOS定位器实现，使用SparkFun OTOS传感器进行定位。
 * 
 * OTOS (Optical Tracking Odometry System) 是SparkFun公司生产的光学跟踪定位系统，
 * 结合了相机、IMU和算法来提供精确的位置和姿态估计。
 */
@Config
public class OTOSLocalizer implements Localizer {
    /**
     * 定位器参数类，用于存储OTOS传感器的配置参数。
     */
    public static class Params {
        /**
         * 角度标量，用于调整角度测量的精度。
         */
        public double angularScalar = 1.0;
        
        /**
         * 线性标量，用于调整线性测量的精度。
         */
        public double linearScalar = 1.0;

        /**
         * OTOS传感器相对于机器人中心的偏移量。
         * 注意：单位为英寸和弧度
         */
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    }

    /**
     * 定位器参数实例，可通过Dashboard进行调整。
     */
    public static Params PARAMS = new Params();

    /**
     * SparkFun OTOS传感器实例，用于获取定位数据。
     */
    public final SparkFunOTOS otos;
    
    /**
     * 当前机器人的姿态估计。
     */
    private Pose2d currentPose;

    /**
     * 构造函数，初始化OTOS定位器。
     * 
     * @param hardwareMap 硬件映射，用于获取OTOS传感器
     * @param initialPose 初始姿态估计
     */
    public OTOSLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        // 获取OTOS传感器实例
        // 注意：确保配置文件中有名称为"sensor_otos"的OTOS设备
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        
        // 设置初始姿态
        currentPose = initialPose;
        // 将初始姿态设置到OTOS传感器
        otos.setPosition(OTOSKt.toOTOSPose(currentPose));
        
        // 设置测量单位
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // 校准IMU
        otos.calibrateImu();
        
        // 应用参数设置
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        otos.setOffset(PARAMS.offset);
    }

    /**
     * 获取当前姿态估计。
     * 
     * @return 定位器当前的姿态估计，包含x、y坐标和heading角度
     */
    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    /**
     * 设置定位器的当前姿态。
     * 
     * @param pose 要设置的目标姿态，包含x、y坐标和heading角度
     */
    @Override
    public void setPose(Pose2d pose) {
        // 更新当前姿态
        currentPose = pose;
        // 将姿态更新到OTOS传感器
        otos.setPosition(OTOSKt.toOTOSPose(currentPose));
    }

    /**
     * 更新定位器的姿态估计。
     * 此方法会从OTOS传感器获取最新的位置、速度和加速度数据，并更新机器人的姿态估计。
     * 
     * @return 定位器当前的速度估计，包含线速度和角速度
     */
    @Override
    public PoseVelocity2d update() {
        // 存储OTOS传感器的姿态、速度和加速度数据
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        
        // 从OTOS传感器获取位置、速度和加速度数据
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);

        // 将OTOS姿态转换为RoadRunner姿态
        currentPose = OTOSKt.toRRPose(otosPose);
        
        // 计算机器人坐标系中的速度
        Vector2d fieldVel = new Vector2d(otosVel.x, otosVel.y);
        Vector2d robotVel = Rotation2d.exp(otosPose.h).inverse().times(fieldVel);
        
        // 返回速度估计
        return new PoseVelocity2d(robotVel, otosVel.h);
    }

    /**
     * 获取AprilTag视觉定位的状态。
     * 由于OTOSLocalizer不使用AprilTag，因此始终返回false。
     * 
     * @return 始终返回false
     */
    @Override
    public boolean getAprilTagStatus() {
        return false;
    }
}
