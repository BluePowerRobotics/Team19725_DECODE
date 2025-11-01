package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.utility.kalmanfilter.OneDimensionKalmanFilter;
import org.firstinspires.ftc.teamcode.utility.kalmanfilter.PosVelTuple;

public class KalmanFusionLocalizer implements Localizer{
    Localizer localizer;
    AprilTagDetector aprilTagDetector;
    OneDimensionKalmanFilter filter_x,filter_y;
    Pose2d pose;
    Pose2d poseError = new Pose2d(0,0,0);
    public KalmanFusionLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose){
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,initialPose);
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap);
        filter_x=new OneDimensionKalmanFilter(initialPose.position.x, 0.0);
        filter_y=new OneDimensionKalmanFilter(initialPose.position.y, 0.0);
        pose=initialPose;
    }
    @Override
    public void setPose(Pose2d pose) {
        update();
        poseError = new Pose2d(pose.position.x-this.pose.position.x,pose.position.y-this.pose.position.y,pose.heading.toDouble()-this.pose.heading.toDouble());
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(pose.position.x+poseError.position.x,pose.position.y+poseError.position.y,pose.heading.toDouble()+poseError.heading.toDouble());
    }

    @Override
    public PoseVelocity2d update() {
        PoseVelocity2d poseVelocity2d = localizer.update();
        Pose2d localizerPose = localizer.getPose();
        pose = Kalman(localizerPose,aprilTagDetector.getPose().pose);
        return poseVelocity2d;
    }

    private Pose2d Kalman(Pose2d wheel, Pose2d AprilTag){
        PosVelTuple result_x=filter_x.Update(wheel.position.x, AprilTag.position.x);
        PosVelTuple result_y=filter_y.Update(wheel.position.y, AprilTag.position.y);
        return new Pose2d(result_x.position,result_y.position,wheel.heading.toDouble());
    }
}
