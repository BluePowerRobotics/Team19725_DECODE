package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.chassis.locate.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
public class ChassisController {

    public static class Params{
        //todo 调整参数
        public double maxV=0.5; // 最大线速度 (m/s)
        public double maxOmega=Math.PI*1/2; // 最大角速度 (rad/s)
        public double zeroThresholdV =0.05; // 速度零点阈值 (m/s)
        public double zeroThresholdOmega =Math.toRadians(0.5); // 角速度零点阈值 (rad/s)
        public double FrontToCenterInch=9.75;
        public double BackToCenterInch=0;
        public double SideToCenterInch=7;
    }
    public static Params PARAMS = new Params();
    public RobotPosition robotPosition;
    HardwareMap hardwareMap;
    boolean useNoHeadMode=false;
    public boolean runningToPoint=false;
    boolean autoLockHeading=true;
    boolean HeadingLockRadianReset=true;
    double HeadingLockRadian;
    public double getHeadingLockRadian(){return HeadingLockRadian;}
    public double noHeadModeStartError;
    public ActionRunner actionRunner=new ActionRunner();
    static ChassisCalculator chassisCalculator= new ChassisCalculator();

    /**
     * 使用上次位置
     */
    public ChassisController(HardwareMap hardwareMap){
        robotPosition= RobotPosition.refresh(hardwareMap);
        this.hardwareMap=hardwareMap;
        HeadingLockRadian = robotPosition.getData().headingRadian;
        noHeadModeStartError=robotPosition.getData().headingRadian;
    }

    /**
     * 初始化时设置位置
     * @param hardwareMap 硬件映射
     * @param initialPosition 初始位置
     * @param initialHeadingRadian 初始朝向，弧度制
     */
    public ChassisController(HardwareMap hardwareMap, Point2D initialPosition, double initialHeadingRadian){
        robotPosition= RobotPosition.refresh(hardwareMap,initialPosition,initialHeadingRadian);
        this.hardwareMap=hardwareMap;
        HeadingLockRadian = robotPosition.getData().headingRadian;
        noHeadModeStartError=robotPosition.getData().headingRadian;
    }
    /**
     * 初始化时设置位置
     * @param hardwareMap 硬件映射
     * @param initialPose 初始位置（前x左y逆时针rotation）
     */
    public ChassisController(HardwareMap hardwareMap, Pose2d initialPose){
        this(hardwareMap,new Point2D(+initialPose.position.y,-initialPose.position.x),initialPose.heading.log());
    }
    public void exchangeNoHeadMode(){
        useNoHeadMode=!useNoHeadMode;
    }
    public boolean getUseNoHeadMode(){
        return useNoHeadMode;
    }
    public void setAutoLockHeading(boolean autoLockHeading){
        this.autoLockHeading=autoLockHeading;
    }
    public void resetNoHeadModeStartError(double Radian){
        noHeadModeStartError = Radian;
    }
    public void resetNoHeadModeStartError(){
        resetNoHeadModeStartError(0);
    }
    public void setHeadingLockRadian(double headingLockRadian){
        chassisCalculator.lastHeadingSetTimeMS = System.currentTimeMillis();
        this.HeadingLockRadian = MathSolver.normalizeAngle(headingLockRadian);
    }

    public void setTargetPoint(Pose2d pose2d){
        runningToPoint = true;
        TrajectoryActionBuilder actionBuilder = robotPosition.mecanumDrive.actionBuilder(robotPosition.getData().getPose2d())
                .strafeToLinearHeading(pose2d.position,pose2d.heading);
        actionRunner.clear();
        actionRunner.add(actionBuilder.build());
        HeadingLockRadian = pose2d.heading.log();
        chassisCalculator.pidRadianHeadLock.reset();
    }
    public boolean ifTargetPointReached(){
        return !actionRunner.isBusy();
    }

    public void resetPosition(Pose2d pose2d){
        robotPosition=RobotPosition.refresh(hardwareMap,pose2d);
    }

    Point2D targetPoint=new Point2D(0,0);
    double targetRadian=0;
    public void gamepadInput(double vx,double vy,double omega){
        vx=vx*PARAMS.maxV;
        vy=vy*PARAMS.maxV;
        omega=omega*PARAMS.maxOmega;
        if(runningToPoint){
            if (Math.abs(Math.hypot(vx,vy))>=PARAMS.zeroThresholdV||Math.abs(omega)>=PARAMS.zeroThresholdOmega) {
                runningToPoint = false;//打断自动驾驶
            }else{
                actionRunner.update();
                if(!actionRunner.isBusy()){
                    runningToPoint = false;
                }
            }
        }
        if(!runningToPoint) {
            if(autoLockHeading){
                if(omega!=0){
                    HeadingLockRadianReset=true;
                }else{
                    if(HeadingLockRadianReset){
                        HeadingLockRadianReset=false;
                        chassisCalculator.firstRunRadian=true;
                        HeadingLockRadian=robotPosition.getData().headingRadian;
                    }
                    if(Math.abs(robotPosition.getData().headingRadian-HeadingLockRadian)<= PARAMS.zeroThresholdOmega) {
                        chassisCalculator.pidRadianHeadLock.reset();
                    }
                    omega=chassisCalculator.calculatePIDRadian(HeadingLockRadian,robotPosition.getData().headingRadian);
                }
            }
            if (useNoHeadMode)
                chassisCalculator.solveGround(vx, vy, omega, robotPosition.getData().headingRadian-noHeadModeStartError);
            else
                chassisCalculator.solveChassis(vx, vy, omega);
        }
    }
}
@Config
class ChassisCalculator {
    public static class Params {
        //todo 调整参数
        public double rkP = 0.8;//radian k
        public double rkI = 1;
        public double rkD = 0.1;
        public double drkP=0.2;
        public double drkI=0;
        public double drkD=0.05;
        public double powerfulPIDRadianUseTimeMS = 1000;
    }
    public static Params PARAMS = new Params();
    long lastHeadingSetTimeMS=0;
    PIDController pidRadianHeadLock;
    PIDController pidRadianDrive;

    ChassisCalculator() {
        // 私有构造函数，防止外部实例化
        pidRadianHeadLock = new PIDController(PARAMS.rkP, PARAMS.rkI, PARAMS.rkD);
        pidRadianDrive = new PIDController(PARAMS.drkP, PARAMS.drkI, PARAMS.drkD);
    }

    /**
     *
     *
     * @param vx    机器人相对于自身的横移速度 (m/s) —— +右
     * @param vy    机器人相对于自身的前进速度 (m/s) —— +前
     * @param omega 机器人旋转角速度 (rad/s) —— +逆时针
     */
    public void solveChassis(double vx, double vy, double omega) {
        RobotPosition.getInstance().mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(vy,-vx),
                omega
        ));
    }

    /**
     * 逆运动学公式（地面坐标系）
     * @param vx 机器人相对于地面的横移速度 (m/s) —— +右
     * @param vy 机器人相对于地面的前进速度 (m/s) —— +前
     * @param omega 机器人旋转角速度 (rad/s) —— +逆时针
     * @param headingRadian 机器人朝向，弧度制
     */
    public void solveGround(double vx, double vy, double omega, double headingRadian) {

        double vxPro = vx * Math.cos(headingRadian) + vy * Math.sin(headingRadian);
        double vyPro = -vx * Math.sin(headingRadian) + vy * Math.cos(headingRadian);
        solveChassis(vxPro, vyPro, omega);
    }
    public void solveGround(double[] vxy,double vOmega,double headingRadian){
        solveGround(vxy[0],vxy[1],vOmega,headingRadian);
    }


    long lastTimeRadian = 0;
    boolean firstRunRadian = true;

    public double calculatePIDRadian(double targetRadian, double currentRadian) {
        if (firstRunRadian) {
            firstRunRadian = false;
            lastTimeRadian = System.nanoTime();
            pidRadianDrive.reset();
        }
        double errorRadian = targetRadian - currentRadian;
        // 归一化到[-π, π]
        errorRadian = MathSolver.normalizeAngle(errorRadian);

        if(System.currentTimeMillis()-lastHeadingSetTimeMS>=PARAMS.powerfulPIDRadianUseTimeMS){
            pidRadianDrive.setPID(PARAMS.drkP, PARAMS.drkI, PARAMS.drkD);
            double output = pidRadianDrive.calculate(errorRadian, 0, (System.nanoTime() - lastTimeRadian) / 1e9);
            lastTimeRadian = System.nanoTime();
            return output;
        }
        pidRadianHeadLock.setPID(PARAMS.rkP, PARAMS.rkI, PARAMS.rkD);
        double output = pidRadianHeadLock.calculate(errorRadian, 0, (System.nanoTime() - lastTimeRadian) / 1e9);
        lastTimeRadian = System.nanoTime();
        return output;
    }
}