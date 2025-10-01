package org.firstinspires.ftc.teamcode.Vision.model;

import com.acmerobotics.roadrunner.Pose2d;

public class AprilTagInfo {
    public Pose2d pose;
    public int id;
    public double distanceToCameraInINCH;
    public double pitchToCameraInDEG;
    //pose用于定位，id，distanceToCameraInINCH，pitchToCameraInDEG用于计算置信度，决定改AprilTag在卡尔曼滤波时置信度系数
    public AprilTagInfo (Pose2d pose, int id, double distanceToCameraInINCH, double pitchToCameraInDEG) {
        this.pose = pose;
        this.id = id;
        this.distanceToCameraInINCH = distanceToCameraInINCH;
        this.pitchToCameraInDEG = pitchToCameraInDEG;
    }
}
