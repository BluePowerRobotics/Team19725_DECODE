package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * 姿态数据类，用于存储从不同传感器获取的机器人姿态信息。
 * 
 * 此类包含两个姿态数据成员，分别存储从AprilTag视觉系统和
 * PinPoint定位系统获取的机器人姿态。
 */
public class PoseData {
    /**
     * 从AprilTag视觉系统获取的机器人姿态。
     * 初始值设置为原点姿态(0, 0, 0)。
     */
    public Pose2d AprilTagPose = new Pose2d(0, 0, 0);
    
    /**
     * 从PinPoint定位系统获取的机器人姿态。
     * 初始值设置为原点姿态(0, 0, 0)。
     */
    public Pose2d PinPointPose = new Pose2d(0, 0, 0);
}
