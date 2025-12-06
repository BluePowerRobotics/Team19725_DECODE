package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.OneDimensionKalmanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.PosVelTuple;

public class KalmanFusionLocalizer implements Localizer{
    Localizer localizer;
    AprilTagDetector aprilTagDetector;
    OneDimensionKalmanFilter filter_x,filter_y;
    HardwareMap hardwareMap;
    double inPerTick;
    Pose2d pose;
    boolean AprilTagStatus = false;
    public KalmanFusionLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose){
        this.hardwareMap=hardwareMap;
        this.inPerTick=inPerTick;
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,initialPose);
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap);
        filter_x=new OneDimensionKalmanFilter(initialPose.position.x, 0.0);
        filter_y=new OneDimensionKalmanFilter(initialPose.position.y, 0.0);
        pose=initialPose;
    }
    @Override
    public void setPose(Pose2d pose) {
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,pose);
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap);
        filter_x=new OneDimensionKalmanFilter(pose.position.x, 0.0);
        filter_y=new OneDimensionKalmanFilter(pose.position.y, 0.0);
        update();
        }

    @Override
    public Pose2d getPose() {
        return new Pose2d(pose.position,pose.heading);
    }

    @Override
    public PoseVelocity2d update() {
        PoseVelocity2d poseVelocity2d = localizer.update();
        Pose2d localizerPose = localizer.getPose();
        pose = Kalman(localizerPose,aprilTagDetector.getPose().pose);
        return poseVelocity2d;
    }


    private Pose2d Kalman(Pose2d wheel, Pose2d AprilTag){
        if(Double.isNaN(AprilTag.position.x)) {
            AprilTagStatus = false;
        }
        else{
            AprilTagStatus = true;
        }
        double distanceX = Math.min(AprilTag.position.x - 72, AprilTag.position.x + 72);
        double distanceY = AprilTag.position.y + 72;
        double distance = Math.sqrt(distanceY * distanceY+ distanceX * distanceX);
        PosVelTuple result_x=filter_x.Update(wheel.position.x, AprilTag.position.x, distance);
        PosVelTuple result_y=filter_y.Update(wheel.position.y, AprilTag.position.y, distance);
        return new Pose2d(result_x.position,result_y.position,wheel.heading.toDouble());
    }
    public boolean getAprilTagStatus(){
        return AprilTagStatus;
    }

}
