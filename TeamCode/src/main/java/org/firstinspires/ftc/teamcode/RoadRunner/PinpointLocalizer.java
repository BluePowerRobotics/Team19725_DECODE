package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

/**
 * GoBilda Pinpoint定位器实现，使用GoBilda Pinpoint Driver硬件进行定位。
 * 
 * Pinpoint Driver是GoBilda公司生产的集成了编码器和IMU的定位硬件，
 * 可以提供更精确的位置和姿态估计。
 */
@Config
public final class PinpointLocalizer implements Localizer {
    /**
     * 定位器参数类，用于存储编码器安装位置的参数。
     * 这些参数用于设置Pinpoint Driver的偏移量。
     */
    public static class Params {
        /**
         * 平行编码器的Y轴位置（以tick为单位）。
         * 表示平行编码器相对于Pinpoint Driver中心的Y轴偏移量。
         */
        public double parYTicks = -2091; // y position of the parallel encoder (in tick units)
        
        /**
         * 垂直编码器的X轴位置（以tick为单位）。
         * 表示垂直编码器相对于Pinpoint Driver中心的X轴偏移量。
         */
        public double perpXTicks = -2240; // x position of the perpendicular encoder (in tick units)
    }

    /**
     * 定位器参数实例，可通过Dashboard进行调整。
     */
    public static Params PARAMS = new Params();

    /**
     * GoBilda Pinpoint Driver实例，用于获取定位数据。
     */
    public final GoBildaPinpointDriver driver;
    
    /**
     * 初始平行编码器方向。
     */
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection;
    
    /**
     * 初始垂直编码器方向。
     */
    public final GoBildaPinpointDriver.EncoderDirection initialPerpDirection;

    /**
     * 世界坐标系到Pinpoint坐标系的变换。
     */
    private Pose2d txWorldPinpoint;
    
    /**
     * Pinpoint坐标系到机器人坐标系的变换。
     */
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    /**
     * 构造函数，初始化Pinpoint定位器。
     * 
     * @param hardwareMap 硬件映射，用于获取Pinpoint Driver
     * @param inPerTick 每个编码器tick对应的英寸数
     * @param initialPose 初始姿态估计
     */
    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // 获取Pinpoint Driver实例
        // 注意：确保配置文件中有名称为"pinpoint"的Pinpoint设备
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // 计算每tick对应的毫米数，并设置编码器分辨率
        // 这段代码非常重要，移除后可能会导致定位异常
        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        // 设置编码器偏移量
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        // 设置编码器方向（根据需要调整）
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        // 应用编码器方向设置
        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        // 重置位置和IMU
        driver.resetPosAndIMU();

        // 设置初始姿态
        txWorldPinpoint = initialPose;
    }

    /**
     * 设置定位器的当前姿态。
     * 
     * @param pose 要设置的目标姿态，包含x、y坐标和heading角度
     */
    @Override
    public void setPose(Pose2d pose) {
        // 更新世界坐标系到Pinpoint坐标系的变换
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    /**
     * 获取当前姿态估计。
     * 
     * @return 定位器当前的姿态估计，包含x、y坐标和heading角度
     */
    @Override
    public Pose2d getPose() {
        // 计算世界坐标系中的机器人姿态
        return txWorldPinpoint.times(txPinpointRobot);
    }

    /**
     * 更新定位器的姿态估计。
     * 此方法会从Pinpoint Driver获取最新的定位数据，并更新机器人的姿态估计。
     * 
     * @return 定位器当前的速度估计，包含线速度和角速度
     */
    @Override
    public PoseVelocity2d update() {
        // 更新Pinpoint Driver的数据
        driver.update();
        
        // 检查Pinpoint Driver是否就绪
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            // 获取Pinpoint坐标系中的机器人姿态
            txPinpointRobot = new Pose2d(
                    driver.getPosX(DistanceUnit.INCH), 
                    driver.getPosY(DistanceUnit.INCH), 
                    driver.getHeading(UnnormalizedAngleUnit.RADIANS)
            );
            
            // 获取世界坐标系中的速度
            Vector2d worldVelocity = new Vector2d(
                    driver.getVelX(DistanceUnit.INCH), 
                    driver.getVelY(DistanceUnit.INCH)
            );
            
            // 将世界坐标系速度转换为机器人坐标系速度
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            // 返回速度估计
            return new PoseVelocity2d(
                    robotVelocity, 
                    driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            );
        }
        
        // 如果Pinpoint Driver未就绪，返回零速度
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    /**
     * 获取AprilTag视觉定位的状态。
     * 由于PinpointLocalizer不使用AprilTag，因此始终返回false。
     * 
     * @return 始终返回false
     */
    @Override
    public boolean getAprilTagStatus() {
        return false;
    }
}
