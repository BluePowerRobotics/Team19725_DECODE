package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/**
 * 定位器接口，定义了所有定位方法必须实现的基本功能。
 * 定位器用于估计机器人在场上的位置和姿态（x, y,  heading）。
 * 
 * 不同的定位实现（如双轮、三轮、视觉定位等）都应该实现此接口，
 * 以提供统一的方法来获取和更新机器人的姿态信息。
 */
public interface Localizer {
    /**
     * 设置定位器的当前姿态。
     * 当需要重置机器人位置或进行初始定位时使用此方法。
     * 
     * @param pose 要设置的目标姿态，包含x、y坐标和heading角度
     */
    void setPose(Pose2d pose);

    /**
     * 获取当前姿态估计。
     * 注意：此方法不会更新姿态估计，只是返回当前缓存的估计值。
     * 必须调用update()方法来更新姿态估计。
     * 
     * @return 定位器当前的姿态估计，包含x、y坐标和heading角度
     */
    Pose2d getPose();

    /**
     * 更新定位器的姿态估计。
     * 此方法会根据传感器数据（如编码器、IMU、视觉等）计算并更新机器人的当前姿态。
     * 
     * @return 定位器当前的速度估计，包含线速度和角速度
     */
    PoseVelocity2d update();
    
    /**
     * 获取AprilTag视觉定位的状态。
     * 用于判断AprilTag视觉定位系统是否正常工作或是否检测到标签。
     * 
     * @return 如果AprilTag定位系统正常工作且检测到标签，则返回true；否则返回false
     */
    boolean getAprilTagStatus();
}
