package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controllers.IMUSensor;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.filter.AngleMeanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.AngleWeightedMeanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.OneDimensionKalmanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.PosVelTuple;

/**
 * 融合定位器，融合了多个IMU传感器和Pinpoint定位器的数据。
 * 
 * 此定位器使用加权平均滤波器来融合多个IMU和Pinpoint的航向数据，
 * 并包含异常检测和修正机制，以提高定位的准确性和稳定性。
 */
@Config
public class FusionLocalizer implements Localizer{
    /**
     * IMU传感器数据的加权平均滤波器。
     */
    AngleWeightedMeanFilter imusFilter = new AngleWeightedMeanFilter(3);
    
    /**
     * 角度均值滤波器数组，用于平滑IMU和Pinpoint的角度数据。
     */
    AngleMeanFilter[] angleMeanFilters = new AngleMeanFilter[]{new AngleMeanFilter(10),new AngleMeanFilter(10),new AngleMeanFilter(10)};
    
    /**
     * 主IMU传感器实例。
     */
    IMUSensor imuSensor;
    
    /**
     * 辅助IMU传感器实例。
     */
    IMUSensor eimuSensor;
    
    /**
     * 内部使用的定位器实例，实际使用PinpointLocalizer进行基本定位。
     */
    Localizer localizer;
    
    /**
     * X轴方向的卡尔曼滤波器（当前未使用）。
     */
    OneDimensionKalmanFilter filter_x,filter_y;
    
    /**
     * 硬件映射，用于获取硬件设备。
     */
    HardwareMap hardwareMap;
    
    /**
     * 每个编码器tick对应的英寸数。
     */
    double inPerTick;
    
    /**
     * 当前机器人的姿态估计。
     */
    Pose2d pose;
    
    /**
     * AprilTag视觉定位的状态（当前未使用，始终为false）。
     */
    boolean AprilTagStatus = false;
    
    /**
     * 主IMU传感器数据是否可信。
     */
    boolean IMUBelievable=true;
    
    /**
     * 辅助IMU传感器数据是否可信。
     */
    boolean EIMUBelievable=true;
    
    /**
     * Pinpoint定位器数据是否可信。
     */
    boolean PINPOINTBelievable=true;
    
    /**
     * 主IMU传感器的角度偏移量，用于将IMU读数转换为世界坐标系。
     */
    double imuAddition = 0;
    
    /**
     * 辅助IMU传感器的角度偏移量，用于将IMU读数转换为世界坐标系。
     */
    double eimuAddition =0;
    
    /**
     * IMU传感器的总角度偏移量（当前未使用）。
     */
    double IMUsAddition = 0;
    
    /**
     * 初始化时的窗口大小，用于计算初始IMU偏移量。
     */
    public static int initialWindowSize =3;

//    // 可配置：一致性与数值离群阈值（原始说明中要求把“不可发生的误差”除以3，乘2作为阈值，因为重心在中线的2/3处）
//    public static double CONSISTENCY_IMPOSSIBLE_ERROR = 0.3; // rad
//    public static double VALUE_IMPOSSIBLE_ERROR = 0.087; // rad
//    public static double CONSISTENCY_DIVISOR = 3.0/2;
//    public static double VALUE_DIVISOR = 3.0/2;
//
//    // 新：要求连续超阈值的帧数才触发重置/修正，避免瞬时噪声导致频繁 reset
//    public static int CONSISTENCY_CONSECUTIVE_REQUIRED = 3;
//    public static int VALUE_CONSECUTIVE_REQUIRED = 3;
//
//    // 计数器
//    int consIMUCount = 0;
//    int consEIMUCount = 0;
//    int consPINCount = 0;
//    int valIMUCount = 0;
//    int valEIMUCount = 0;
//    int valPINCount = 0;

    /**
     * 主IMU传感器的丢失时间计数器。
     */
    int IMULostTime =0;
    
    /**
     * 辅助IMU传感器的丢失时间计数器。
     */
    int EIMULostTime =0;
    
    /**
     * Pinpoint定位器的丢失时间计数器。
     */
    int PINPOINTLostTime=0;
    
    /**
     * IMU传感器的最大丢失时间阈值。
     */
    public static int LostMaxTime=40;
    
    /**
     * Pinpoint定位器的最大丢失时间阈值。
     */
    public static int PinPointLostMaxTime=40;
    
    /**
     * 混合使用的最大时间阈值。
     */
    public static int MixMaxTime=20;
    
    /**
     * 主IMU传感器的权重。
     */
    public static double IMUWeight = 0.1;
    
    /**
     * 辅助IMU传感器的权重。
     */
    public static double EIMUWeight = 20;
    
    /**
     * Pinpoint定位器的权重。
     */
    public static double PINPWeight = 1;
    
    /**
     * 主IMU传感器的上次读数。
     */
    double IMULastValue =0;
    
    /**
     * 辅助IMU传感器的上次读数。
     */
    double EIMULastValue =0;
    
    /**
     * Pinpoint定位器的上次读数。
     */
    double PINPOINTLastValue=0;
    
    /**
     * 融合后的角度值。
     */
    double fusedAngle = 0;
    
    /**
     * IMU滤波器，用于平滑IMU数据（当前未使用）。
     */
    AngleMeanFilter imuFilter = new AngleMeanFilter(10);
    
    /**
     * AprilTag滤波器，用于平滑AprilTag数据（当前未使用）。
     */
    AngleMeanFilter aprilTagFilter = new AngleMeanFilter(10);
    
    /**
     * 误差滤波器，用于平滑误差数据（当前未使用）。
     */
    AngleMeanFilter errorFilter = new AngleMeanFilter(20);

    /**
     * 构造函数，初始化融合定位器。
     * 
     * @param hardwareMap 硬件映射，用于获取硬件设备
     * @param inPerTick 每个编码器tick对应的英寸数
     * @param initialPose 初始姿态估计
     */
    public FusionLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose){
        // 初始化辅助IMU传感器
        eimuSensor = new IMUSensor(hardwareMap, "eimu", new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(90,-(180-Math.toDegrees(Math.atan2(3,4))),0)));
        // 初始化主IMU传感器
        imuSensor = new IMUSensor(hardwareMap, "imu", new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        
        // 检查主IMU传感器是否初始化成功
        if (!imuSensor.ifInitiated()){
            imuSensor.reset();
            IMUBelievable=false;
        }
        
        // 检查辅助IMU传感器是否初始化成功
        if (!eimuSensor.ifInitiated()){
            imuSensor.reset();
            EIMUBelievable=false;
        }
        
        // 初始化IMU角度滤波器，用于计算初始偏移量
        AngleMeanFilter  imuInit = new AngleMeanFilter(initialWindowSize);
        AngleMeanFilter eimuInit = new AngleMeanFilter(initialWindowSize);
        
        // 收集初始IMU数据
        int i =0;
        while(i<initialWindowSize){
            imuInit.filter(imuSensor.getYaw(AngleUnit.RADIANS));
            eimuInit.filter(eimuSensor.getYaw(AngleUnit.RADIANS));
            i++;
        }
        
        // 计算IMU传感器的初始偏移量
        imuAddition  = MathSolver.normalizeAngle(initialPose.heading.log()-imuInit.getAverageAngle());
        eimuAddition = MathSolver.normalizeAngle(initialPose.heading.log()-eimuInit.getAverageAngle());
        
        // 设置初始融合角度
        fusedAngle = initialPose.heading.log();
        
        // 存储硬件映射和编码器参数
        this.hardwareMap=hardwareMap;
        this.inPerTick=inPerTick;
        
        // 初始化PinpointLocalizer作为基础定位器
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,initialPose);
        
        // 初始化卡尔曼滤波器（当前未使用）
        filter_x=new OneDimensionKalmanFilter(initialPose.position.x, 0.0);
        filter_y=new OneDimensionKalmanFilter(initialPose.position.y, 0.0);
        
        // 初始化角度滤波器
        angleMeanFilters[0].filter(initialPose.heading.log());
        angleMeanFilters[1].filter(initialPose.heading.log());
        angleMeanFilters[2].filter(initialPose.heading.log());
        
        // 设置初始姿态
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
        fusedAngle = pose.heading.log();
        imuAddition = pose.heading.log()-imuSensor.getYaw(AngleUnit.RADIANS);
        eimuAddition = pose.heading.log()-eimuSensor.getYaw(AngleUnit.RADIANS);
        imusFilter = new AngleWeightedMeanFilter(3);
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,pose);
        //aprilTagDetector = new AprilTagDetector();
        //aprilTagDetector.init(hardwareMap);
        filter_x=new OneDimensionKalmanFilter(pose.position.x, 0.0);
        filter_y=new OneDimensionKalmanFilter(pose.position.y, 0.0);
        this.pose=pose;
        angleMeanFilters[0].reset();
        angleMeanFilters[1].reset();
        angleMeanFilters[2].reset();
        angleMeanFilters[0].filter(pose.heading.log());
        angleMeanFilters[1].filter(pose.heading.log());
        angleMeanFilters[2].filter(pose.heading.log());
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

//        // 调用离群检测与修正
//        detectAndCorrectOutliers(IMUHeading, EIMUHeading, localizerHeading, heading);

        fixLost(heading);
        return poseVelocity2d;
    }


    private Pose2d Kalman(Pose2d wheel, Pose2d AprilTag){
        AprilTagStatus = !Double.isNaN(AprilTag.position.x);
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
    public static int LostMaxTime=40;
    public static int PinPointLostMaxTime=40;
    public static int MixMaxTime=20;
    public static double IMUWeight = 0.1;
    public static double EIMUWeight = 20;
    public static double PINPWeight = 1;
    double IMULastValue =0;
    double EIMULastValue =0;
    double PINPOINTLastValue=0;
    double fusedAngle = 0;
    private double fusionIMU(double IMU, double EIMU, double pinpoint){
        if(!Double.isNaN(IMU)){
            if(IMULastValue ==IMU){
                IMULostTime++;
            } else {
                IMULastValue = IMU;
                IMULostTime = 0;
            }
        }else{
            IMULostTime++;
        }
        if(!Double.isNaN(EIMU)){
            if(EIMULastValue ==EIMU){
                EIMULostTime++;
            } else {
                EIMULastValue = EIMU;
                EIMULostTime = 0;
            }
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
            PINPOINTLostTime+=10;
        }
         IMUBelievable = !( IMULostTime>=LostMaxTime);
        EIMUBelievable = !(EIMULostTime>=LostMaxTime);
        PINPOINTBelievable = !(PINPOINTLostTime>=PinPointLostMaxTime);
        imusFilter.reset();
        imusFilter.filter(imuAddition+IMU,  !(IMULostTime>=MixMaxTime)?IMUWeight:0.0);
        imusFilter.filter(eimuAddition+EIMU, !(EIMULostTime>=MixMaxTime)?EIMUWeight:0.0);
        if(!Double.isNaN(imusFilter.filter(pinpoint, !(PINPOINTLostTime>=MixMaxTime)?PINPWeight:0.0))){
            fusedAngle = imusFilter.getAverageAngle();
        }
        InstanceTelemetry.getTelemetry().addData("IMU ",MathSolver.normalizeAngle(IMU+imuAddition));
        InstanceTelemetry.getTelemetry().addData("EIMU",MathSolver.normalizeAngle(EIMU+eimuAddition));
        InstanceTelemetry.getTelemetry().addData("PINP",MathSolver.normalizeAngle(pinpoint));
        return fusedAngle;
    }
    private void fixLost(double fusedAngle){
        if(!IMUBelievable){
            imuSensor.reset();
            imuAddition=fusedAngle-imuSensor.getYaw(AngleUnit.RADIANS);
            IMULostTime=MixMaxTime;
        }
        if(!EIMUBelievable){
            eimuSensor.reset();
            eimuAddition = fusedAngle-eimuSensor.getYaw(AngleUnit.RADIANS);
            EIMULostTime=MixMaxTime;
        }
        if(!PINPOINTBelievable){
            localizer = new PinpointLocalizer(hardwareMap,inPerTick,new Pose2d(pose.position,fusedAngle) );
            PINPOINTLostTime=MixMaxTime;
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

//    // --- 新增：一致性与数值离群检测与修正 ---
//    private void detectAndCorrectOutliers(double imuRaw, double eimuRaw, double pinpointRaw, double fusedHeading){
//        double consThreshold = CONSISTENCY_IMPOSSIBLE_ERROR / CONSISTENCY_DIVISOR;
//        double valThreshold = VALUE_IMPOSSIBLE_ERROR / VALUE_DIVISOR;
//
//        double meanIMU = Double.NaN;
//        double meanEIMU = Double.NaN;
//        double meanPIN = Double.NaN;
//        try{
//            if(!Double.isNaN(imuRaw)) meanIMU = angleMeanFilters[0].filter(MathSolver.normalizeAngle(imuRaw + imuAddition));
//        }catch (Exception ignored){}
//        try{
//            if(!Double.isNaN(eimuRaw)) meanEIMU = angleMeanFilters[1].filter(MathSolver.normalizeAngle(eimuRaw + eimuAddition));
//        }catch (Exception ignored){}
//        try{
//            if(!Double.isNaN(pinpointRaw)) meanPIN = angleMeanFilters[2].filter(MathSolver.normalizeAngle(pinpointRaw));
//        }catch (Exception ignored){}
//
//        double consIMU = angleMeanFilters[0].getConsistency();
//        double consEIMU = angleMeanFilters[1].getConsistency();
//        double consPIN = angleMeanFilters[2].getConsistency();
//
//        InstanceTelemetry.getTelemetry().addData("ConsIMU", consIMU);
//        InstanceTelemetry.getTelemetry().addData("ConsEIMU", consEIMU);
//        InstanceTelemetry.getTelemetry().addData("ConsPIN", consPIN);
//
//        // 一致性离群：需要连续多帧
//        if(consIMU < consThreshold){
//            consIMUCount++;
//        } else consIMUCount = 0;
//        if(consEIMU < consThreshold){
//            consEIMUCount++;
//        } else consEIMUCount = 0;
//        if(consPIN < consThreshold){
//            consPINCount++;
//        } else consPINCount = 0;
//
//        if(consIMUCount >= CONSISTENCY_CONSECUTIVE_REQUIRED){
//            InstanceTelemetry.getTelemetry().addData("Outlier","IMU consistency low (consecutive), resetting");
//            try{
//                imuSensor.reset();
//                imuAddition = fusedHeading - imuSensor.getYaw(AngleUnit.RADIANS);
//            }catch (Exception e){
//                IMUBelievable = false;
//            }
//            IMULostTime = MixMaxTime;
//            consIMUCount = 0; // 已处理，清零计数
//        }
//        if(consEIMUCount >= CONSISTENCY_CONSECUTIVE_REQUIRED){
//            InstanceTelemetry.getTelemetry().addData("Outlier","EIMU consistency low (consecutive), resetting");
//            try{
//                eimuSensor.reset();
//                eimuAddition = fusedHeading - eimuSensor.getYaw(AngleUnit.RADIANS);
//            }catch (Exception e){
//                EIMUBelievable = false;
//            }
//            EIMULostTime = MixMaxTime;
//            consEIMUCount = 0;
//        }
//        if(consPINCount >= CONSISTENCY_CONSECUTIVE_REQUIRED){
//            InstanceTelemetry.getTelemetry().addData("Outlier","Pinpoint consistency low (consecutive), re-init localizer");
//            localizer = new PinpointLocalizer(hardwareMap,inPerTick,new Pose2d(pose.position,fusedHeading));
//            PINPOINTLostTime = MixMaxTime;
//            PINPOINTBelievable = false;
//            consPINCount = 0;
//        }
//
//        // 数值离群：用现有可用均值的中位/平均作为参考
//        double[] vals = new double[]{meanIMU, meanEIMU, meanPIN};
//        double[] tmp = new double[3];
//        int n=0;
//        for(double v:vals) if(!Double.isNaN(v)) tmp[n++]=v;
//        double ref = Double.NaN;
//        if(n>0){
//            if(n==1) ref = tmp[0];
//            else if(n==2) ref = MathSolver.normalizeAngle((tmp[0]+tmp[1])/2.0);
//            else {
//                double a=tmp[0], b=tmp[1], c=tmp[2];
//                if(a>b){double t=a;a=b;b=t;}
//                if(b>c){double t=b;b=c;c=t; if(a>b){double t2=a;a=b;b=t2;}}
//                ref = b;
//            }
//        }
//
//        if(!Double.isNaN(ref)){
//            // IMU
//            if(!Double.isNaN(meanIMU)){
//                double diff = Math.abs(MathSolver.normalizeAngle(meanIMU - ref));
//                if(diff > valThreshold) valIMUCount++; else valIMUCount = 0;
//                if(valIMUCount >= VALUE_CONSECUTIVE_REQUIRED){
//                    InstanceTelemetry.getTelemetry().addData("ValueOutlier","IMU deviates (consecutive) by "+diff+", correcting");
//                    try{
//                        imuSensor.reset();
//                        imuAddition = ref - imuSensor.getYaw(AngleUnit.RADIANS);
//                    }catch (Exception e){
//                        IMUBelievable = false;
//                    }
//                    IMULostTime = MixMaxTime;
//                    valIMUCount = 0;
//                }
//            }
//            // EIMU
//            if(!Double.isNaN(meanEIMU)){
//                double diff = Math.abs(MathSolver.normalizeAngle(meanEIMU - ref));
//                if(diff > valThreshold) valEIMUCount++; else valEIMUCount = 0;
//                if(valEIMUCount >= VALUE_CONSECUTIVE_REQUIRED){
//                    InstanceTelemetry.getTelemetry().addData("ValueOutlier","EIMU deviates (consecutive) by "+diff+", correcting");
//                    try{
//                        eimuSensor.reset();
//                        eimuAddition = ref - eimuSensor.getYaw(AngleUnit.RADIANS);
//                    }catch (Exception e){
//                        EIMUBelievable = false;
//                    }
//                    EIMULostTime = MixMaxTime;
//                    valEIMUCount = 0;
//                }
//            }
//            // Pinpoint
//            if(!Double.isNaN(meanPIN)){
//                double diff = Math.abs(MathSolver.normalizeAngle(meanPIN - ref));
//                if(diff > valThreshold) valPINCount++; else valPINCount = 0;
//                if(valPINCount >= VALUE_CONSECUTIVE_REQUIRED){
//                    InstanceTelemetry.getTelemetry().addData("ValueOutlier","Pinpoint deviates (consecutive) by "+diff+", re-init localizer");
//                    localizer = new PinpointLocalizer(hardwareMap,inPerTick,new Pose2d(pose.position,ref));
//                    PINPOINTLostTime = MixMaxTime;
//                    PINPOINTBelievable = false;
//                    valPINCount = 0;
//                }
//            }
//        }
//    }
}
