package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import java.io.IOException;

public class WriteInstantlyLocalizer implements Localizer{
    Localizer localizer;
    Pose2d pose;
    boolean AprilTagStatus = false;
    public WriteInstantlyLocalizer(Localizer localizer){
        this.localizer =localizer;
        pose=localizer.getPose();
    }
    @Override
    public void setPose(Pose2d pose) {
        localizer.setPose(pose);
        }

    @Override
    public Pose2d getPose() {
        return new Pose2d(pose.position,pose.heading);
    }

    @Override
    public PoseVelocity2d update() {
        PoseVelocity2d poseVelocity2d = localizer.update();
        pose = localizer.getPose();
        try (java.io.FileWriter writer = new java.io.FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(pose.position.x + "," +
                    pose.position.y + "," +
                    pose.heading.toDouble());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return poseVelocity2d;
    }

    @Override
    public boolean getAprilTagStatus() {
        return false;
    }


}
