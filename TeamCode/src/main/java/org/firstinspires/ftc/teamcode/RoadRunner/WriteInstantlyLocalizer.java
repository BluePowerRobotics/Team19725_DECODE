package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;

/**
 * 即时写入定位器，是对PinpointLocalizer的包装，
 * 每次更新定位时会将当前姿态信息写入到SD卡上的文件中。
 * 
 * 此定位器还包含异常处理机制，当姿态信息中包含NaN值时，会重新初始化PinpointLocalizer。
 */
public class WriteInstantlyLocalizer implements Localizer{
    /**
     * 内部使用的定位器实例，实际使用PinpointLocalizer进行定位。
     */
    Localizer localizer;
    
    /**
     * 当前机器人的姿态估计。
     */
    Pose2d pose;
    
    /**
     * 每个编码器tick对应的英寸数。
     */
    double inPerTick;
    
    /**
     * 硬件映射，用于获取硬件设备。
     */
    HardwareMap hardwareMap;
    
    /**
     * AprilTag视觉定位的状态（当前未使用，始终为false）。
     */
    boolean AprilTagStatus = false;
    
    /**
     * 构造函数，初始化即时写入定位器。
     * 
     * @param hardwareMap 硬件映射，用于获取硬件设备
     * @param inPerTick 每个编码器tick对应的英寸数
     * @param initialPose 初始姿态估计
     */
    public WriteInstantlyLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose){
        this.hardwareMap=hardwareMap;
        this.inPerTick=inPerTick;
        // 初始化内部使用的PinpointLocalizer
        this.localizer =new PinpointLocalizer(hardwareMap,inPerTick,initialPose);
        // 获取初始姿态
        pose=localizer.getPose();
    }
    
    /**
     * 设置定位器的当前姿态。
     * 
     * @param pose 要设置的目标姿态，包含x、y坐标和heading角度
     */
    @Override
    public void setPose(Pose2d pose) {
        // 委托给内部定位器处理
        localizer.setPose(pose);
    }

    /**
     * 获取当前姿态估计。
     * 
     * @return 当前姿态估计，包含x、y坐标和heading角度
     */
    @Override
    public Pose2d getPose() {
        // 返回当前姿态的副本
        return new Pose2d(pose.position,pose.heading);
    }

    /**
     * 更新定位器的姿态估计，并将姿态信息写入到SD卡文件中。
     * 
     * @return 定位器当前的速度估计，包含线速度和角速度
     */
    @Override
    public PoseVelocity2d update() {
        // 更新内部定位器的姿态
        PoseVelocity2d poseVelocity2d = localizer.update();
        // 获取更新后的姿态
        Pose2d pose = localizer.getPose();
        
        // 检查姿态是否包含NaN值（无效值）
        if(Double.isNaN(pose.position.x)||Double.isNaN(pose.position.y)||Double.isNaN(pose.heading.toDouble())){
            // 如果包含NaN值，重新初始化PinpointLocalizer
            localizer = new PinpointLocalizer(hardwareMap,inPerTick,this.pose);
            // 重新更新姿态
            poseVelocity2d = localizer.update();
            pose = localizer.getPose();
        }
        
        // 更新当前姿态
        this.pose = pose;
        
        // 将姿态信息写入到SD卡上的pose.txt文件
        try (java.io.FileWriter writer = new java.io.FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(pose.position.x + "," +
                    pose.position.y + "," +
                    pose.heading.toDouble());
        } catch (IOException e) {
            // 如果写入失败，抛出运行时异常
            throw new RuntimeException(e);
        }
        
        // 返回速度估计
        return poseVelocity2d;
    }

    /**
     * 获取AprilTag视觉定位的状态。
     * 注意：当前未使用AprilTag，始终返回false。
     * 
     * @return 始终返回false
     */
    @Override
    public boolean getAprilTagStatus() {
        return false;
    }


}
