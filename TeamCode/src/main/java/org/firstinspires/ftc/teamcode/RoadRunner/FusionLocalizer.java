package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.controllers.IMUSensor;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.filter.AngleMeanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.AngleWeightedMeanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.OneDimensionKalmanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.PosVelTuple;
@Config
public class FusionLocalizer implements Localizer{
    AngleWeightedMeanFilter imusFilter = new AngleWeightedMeanFilter(3);
    IMUSensor imuSensor;
    IMUSensor eimuSensor;
    Localizer localizer;
    //AprilTagDetector aprilTagDetector;
    OneDimensionKalmanFilter filter_x,filter_y;
    HardwareMap hardwareMap;
    double inPerTick;
    Pose2d pose;
    boolean AprilTagStatus = false;
    boolean IMUBelievable=true;
    boolean EIMUBelievable=true;
    boolean PINPOINTBelievable=true;
    double imuAddition = 0;
    double eimuAddition =0;
    double IMUsAddition = 0;
    public FusionLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose){
        eimuSensor = new IMUSensor(hardwareMap, "eimu", new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(90,-(180-Math.toDegrees(Math.atan2(3,4))),0)));
         imuSensor = new IMUSensor(hardwareMap, "imu", new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        if (!imuSensor.ifInitiated()){
            imuSensor.reset();
            IMUBelievable=false;
        }
        if (!eimuSensor.ifInitiated()){
            imuSensor.reset();
            EIMUBelievable=false;
        }
        imuAddition  = MathSolver.normalizeAngle(initialPose.heading.log()- imuSensor.getYaw(AngleUnit.RADIANS));
        eimuAddition = MathSolver.normalizeAngle(initialPose.heading.log()-eimuSensor.getYaw(AngleUnit.RADIANS));
        this.hardwareMap=hardwareMap;
        this.inPerTick=inPerTick;
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,initialPose);
        //aprilTagDetector = new AprilTagDetector();
        //aprilTagDetector.init(hardwareMap);
        filter_x=new OneDimensionKalmanFilter(initialPose.position.x, 0.0);
        filter_y=new OneDimensionKalmanFilter(initialPose.position.y, 0.0);
        pose=initialPose;
    }
    @Override
    public void setPose(Pose2d pose) {
        try {
            imuSensor.reset();
        } catch (Exception e) {
            InstanceTelemetry.getTelemetry().addLine(e.getMessage());
            IMUBelievable=false;
        }
        try {
            eimuSensor.reset();
        } catch (Exception e) {
            InstanceTelemetry.getTelemetry().addLine(e.getMessage());
            EIMUBelievable=false;
        }
        imuAddition = pose.heading.log();
        eimuAddition = pose.heading.log();
        imusFilter = new AngleWeightedMeanFilter(3);
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,pose);
        //aprilTagDetector = new AprilTagDetector();
        //aprilTagDetector.init(hardwareMap);
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
        //Pose2d aprilTagPose = aprilTagDetector.getPose().pose;
        double localizerHeading=localizerPose.heading.log();
        //double aprilTagHeading = aprilTagPose.heading.log();
        double IMUHeading=Double.NaN;
        double EIMUHeading=Double.NaN;
        try{
            IMUHeading = imuSensor.getYaw(AngleUnit.RADIANS);
            EIMUHeading= eimuSensor.getYaw(AngleUnit.RADIANS);
        } catch (Exception e) {
            InstanceTelemetry.getTelemetry().addLine(e.getMessage());
        }
        double heading = fusionIMU(IMUHeading, EIMUHeading, localizerHeading);
        localizerPose = new Pose2d(localizerPose.position, MathSolver.normalizeAngle(heading+IMUsAddition));
        pose = /*Kalman(*/localizerPose/*,aprilTagPose)*/;
        fixLost(heading);
        return poseVelocity2d;
    }


    private Pose2d Kalman(Pose2d wheel, Pose2d AprilTag){
        if(Double.isNaN(AprilTag.position.x)) {
            AprilTagStatus = false;
        }
        else{
            AprilTagStatus = true;
        }
//        double distanceX = Math.min(AprilTag.position.x - 72, AprilTag.position.x + 72);
//        double distanceY = AprilTag.position.y + 72;
//        double distance = Math.sqrt(distanceY * distanceY+ distanceX * distanceX);
        PosVelTuple result_x=filter_x.Update(wheel.position.x, AprilTag.position.x);
        PosVelTuple result_y=filter_y.Update(wheel.position.y, AprilTag.position.y);
        return new Pose2d(result_x.position,result_y.position,wheel.heading.toDouble());
    }
    public boolean getAprilTagStatus(){
        return AprilTagStatus;
    }
    int IMULostTime =0;
    int EIMULostTime =0;
    int PINPOINTLostTime=0;
    public static int LostMaxTime=200;
    public static int PinPointLostMaxTime=200;
    public static int MixMaxTime=5;

    double IMULastValue =0;
    double EIMULastValue =0;
    double PINPOINTLastValue=0;
    private double fusionIMU(double IMU, double EIMU, double pinpoint){
        if(!Double.isNaN(IMU)){
//            if(IMULastValue ==IMU){
//                IMULostTime++;
//            } else {
                IMULastValue = IMU;
                IMULostTime = 0;
//            }
        }else{
            IMULostTime++;
        }
        if(!Double.isNaN(EIMU)){
//            if(EIMULastValue ==EIMU){
//                EIMULostTime++;
//            } else {
                EIMULastValue = EIMU;
                EIMULostTime = 0;
//            }
        }else{
            EIMULostTime++;
        }
        if(!Double.isNaN(pinpoint)) {
            if (PINPOINTLastValue == pinpoint) {
                PINPOINTLostTime++;
            } else {
                PINPOINTLastValue = pinpoint;
                PINPOINTLostTime = 0;
            }
        }else{
            PINPOINTLostTime++;
        }

         IMUBelievable = !( IMULostTime>=LostMaxTime);
        EIMUBelievable = !(EIMULostTime>=LostMaxTime);
        PINPOINTBelievable = !(PINPOINTLostTime>=PinPointLostMaxTime);
        imusFilter.reset();
        imusFilter.filter(imuAddition+IMU,  !(IMULostTime>=MixMaxTime)?1.0:0.0);
        imusFilter.filter(eimuAddition+EIMU, !(EIMULostTime>=MixMaxTime)?1.0:0.0);
        double fusedAngle=imusFilter.filter(pinpoint, !(PINPOINTLostTime>=MixMaxTime)?2.0:0.0);
        //InstanceTelemetry.getTelemetry().addData(" IMUBelievable",IMUBelievable);
        //InstanceTelemetry.getTelemetry().addData("EIMUBelievable",EIMUBelievable);
        //InstanceTelemetry.getTelemetry().addData("PINPBelievable",PINPOINTBelievable);
        InstanceTelemetry.getTelemetry().addData("IMU ",MathSolver.normalizeAngle(IMU+imuAddition));
        //InstanceTelemetry.getTelemetry().addData("IMU_origin",MathSolver.normalizeAngle(IMU));
        InstanceTelemetry.getTelemetry().addData("IMU_addition",MathSolver.normalizeAngle(imuAddition));
        InstanceTelemetry.getTelemetry().addData("EIMU",MathSolver.normalizeAngle(EIMU+eimuAddition));
        //InstanceTelemetry.getTelemetry().addData("EIMU_origin",MathSolver.normalizeAngle(EIMU));
        InstanceTelemetry.getTelemetry().addData("EIMU_addition",MathSolver.normalizeAngle(eimuAddition));
        InstanceTelemetry.getTelemetry().addData("PINP",MathSolver.normalizeAngle(pinpoint));
        return fusedAngle;
    }
    private void fixLost(double fusedAngle){
        if(!IMUBelievable){
            imuSensor.reset();
            imuAddition=fusedAngle;
            IMULostTime=0;
        }
        if(!EIMUBelievable){
            eimuSensor.reset();
            eimuAddition = fusedAngle;
            EIMULostTime=0;
        }
        if(!PINPOINTBelievable){
            localizer = new PinpointLocalizer(hardwareMap,inPerTick,new Pose2d(pose.position,fusedAngle) );
            PINPOINTLostTime=0;
        }
    }
    AngleMeanFilter imuFilter = new AngleMeanFilter(10);
    AngleMeanFilter aprilTagFilter = new AngleMeanFilter(10);
    AngleMeanFilter errorFilter = new AngleMeanFilter(20);
    private void fusionAprilTag(double IMU, double AprilTag){
        double fusedIMU = imuFilter.filter(IMU);
        double fusedAprilTag = aprilTagFilter.filter(AprilTag);
        if(aprilTagFilter.getConsistency()>=0.9&&imuFilter.getConsistency()>=0.9) {
            double error = MathSolver.normalizeAngle(fusedAprilTag - fusedIMU);
            errorFilter.filter(error);
        }
        IMUsAddition = errorFilter.getAverageAngle();
    }
}
