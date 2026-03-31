package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.OneDimensionKalmanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.PosVelTuple;

/**
 * 卡尔曼滤波融合定位器，融合了Pinpoint定位器和AprilTag视觉定位的数据。
 * 
 * 此定位器使用卡尔曼滤波器来融合两种定位方法的数据，
 * 以获得更准确、更稳定的位置和姿态估计。
 */
public class KalmanFusionLocalizer implements Localizer{
    /**
     * 内部使用的定位器实例，实际使用PinpointLocalizer进行基本定位。
     */
    Localizer localizer;
    
    /**
     * AprilTag检测器，用于获取视觉定位数据。
     */
    AprilTagDetector aprilTagDetector;
    
    /**
     * X轴方向的卡尔曼滤波器。
     */
    OneDimensionKalmanFilter filter_x;
    
    /**
     * Y轴方向的卡尔曼滤波器。
     */
    OneDimensionKalmanFilter filter_y;
    
    /**
     * 硬件映射，用于获取硬件设备。
     */
    HardwareMap hardwareMap;
    
    /**
     * 每个编码器tick对应的英寸数。
     */
    double inPerTick;
    
    /**
     * 当前机器人的姿态估计（融合后的数据）。
     */
    Pose2d pose;
    
    /**
     * 从AprilTag获取的姿态估计。
     */
    Pose2d AprilTagPose;
    
    /**
     * 从本地定位器获取的姿态估计。
     */
    Pose2d localizerPose;
    
    /**
     * AprilTag视觉定位的状态，true表示检测到AprilTag，false表示未检测到。
     */
    boolean AprilTagStatus = false;
    
    /**
     * 姿态数据存储对象，用于存储不同定位方法的姿态数据。
     */
    PoseData poseData;
    
    /**
     * 构造函数，初始化卡尔曼滤波融合定位器。
     * 
     * @param hardwareMap 硬件映射，用于获取硬件设备
     * @param inPerTick 每个编码器tick对应的英寸数
     * @param initialPose 初始姿态估计
     * @param poseData 姿态数据存储对象
     */
    public KalmanFusionLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose,PoseData poseData){
        this.hardwareMap=hardwareMap;
        this.inPerTick=inPerTick;
        // 初始化PinpointLocalizer作为基础定位器
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,initialPose);
        // 初始化AprilTag检测器
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap);
        // 初始化X轴和Y轴的卡尔曼滤波器
        filter_x=new OneDimensionKalmanFilter(initialPose.position.x, 0.0);
        filter_y=new OneDimensionKalmanFilter(initialPose.position.y, 0.0);
        // 设置初始姿态
        pose=initialPose;
    }
    
    /**
     * 设置定位器的当前姿态。
     * 
     * @param pose 要设置的目标姿态，包含x、y坐标和heading角度
     */
    @Override
    public void setPose(Pose2d pose) {
        // 重新初始化PinpointLocalizer
        localizer = new PinpointLocalizer(hardwareMap,inPerTick,pose);
        // 重新初始化AprilTag检测器
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap);
        // 重新初始化卡尔曼滤波器
        filter_x=new OneDimensionKalmanFilter(pose.position.x, 0.0);
        filter_y=new OneDimensionKalmanFilter(pose.position.y, 0.0);
        // 更新姿态
        update();
    }

    /**
     * 获取当前姿态估计。
     * 
     * @return 定位器当前的姿态估计，包含x、y坐标和heading角度
     */
    @Override
    public Pose2d getPose() {
        // 返回当前姿态的副本
        return new Pose2d(pose.position,pose.heading);
    }

    /**
     * 更新定位器的姿态估计。
     * 此方法会从PinpointLocalizer和AprilTag获取定位数据，然后使用卡尔曼滤波器进行融合。
     * 
     * @return 定位器当前的速度估计，包含线速度和角速度
     */
    @Override
    public PoseVelocity2d update() {
        // 更新PinpointLocalizer并获取速度估计
        PoseVelocity2d poseVelocity2d = localizer.update();
        // 获取PinpointLocalizer的姿态估计
        localizerPose = localizer.getPose();
        // 存储PinpointLocalizer的姿态数据
        poseData.PinPointPose=localizerPose;
        // 获取AprilTag的姿态估计
        AprilTagPose = aprilTagDetector.getPose().pose;
        // 存储AprilTag的姿态数据
        poseData.AprilTagPose=AprilTagPose;
        // 使用卡尔曼滤波器融合两种定位数据
        pose = Kalman(localizerPose,AprilTagPose);
        // 返回速度估计
        return poseVelocity2d;
    }

    /**
     * 使用卡尔曼滤波器融合两种定位数据。
     * 
     * @param wheel 从轮式定位器获取的姿态
     * @param AprilTag 从AprilTag获取的姿态
     * @return 融合后的姿态估计
     */
    private Pose2d Kalman(Pose2d wheel, Pose2d AprilTag){
        // 检查AprilTag姿态是否有效
        if(Double.isNaN(AprilTag.position.x)) {
            // 如果AprilTag姿态无效，设置状态为false
            AprilTagStatus = false;
        }
        else{
            // 如果AprilTag姿态有效，设置状态为true
            AprilTagStatus = true;
        }
        
        // 使用卡尔曼滤波器融合X轴位置数据
        PosVelTuple result_x=filter_x.Update(wheel.position.x, AprilTag.position.x);
        // 使用卡尔曼滤波器融合Y轴位置数据
        PosVelTuple result_y=filter_y.Update(wheel.position.y, AprilTag.position.y);
        
        // 返回融合后的姿态（使用轮式定位器的heading，因为AprilTag可能没有提供或精度不够）
        return new Pose2d(result_x.position,result_y.position,wheel.heading.toDouble());
    }
    
    /**
     * 获取AprilTag视觉定位的状态。
     * 
     * @return 如果检测到AprilTag，则返回true；否则返回false
     */
    public boolean getAprilTagStatus(){
        return AprilTagStatus;
    }
}
