package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.chassis.locate.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.filter.AngleMeanFilter;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
public class ChassisController {

    public static class Params{
        //todo 调整参数
        public double maxV=0.5; // 最大线速度 (m/s)
        public double maxA=0.5; // 最大加速度 (m/s²)
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
    boolean fullyAutoMode = false;
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
     * OpMode初始化时调用
     */
    public ChassisController(HardwareMap hardwareMap){
        robotPosition= RobotPosition.refresh(hardwareMap);
        this.hardwareMap=hardwareMap;
        HeadingLockRadian = robotPosition.getData().headingRadian;
        noHeadModeStartError=robotPosition.getData().headingRadian;
    }

    /**
     * Auto初始化时调用
     * @param hardwareMap 硬件映射
     * @param initialPosition 初始位置
     * @param initialHeadingRadian 初始朝向，弧度制
     */
    public ChassisController(HardwareMap hardwareMap, Point2D initialPosition, double initialHeadingRadian){
        robotPosition= RobotPosition.refresh(hardwareMap,initialPosition,initialHeadingRadian);
        this.hardwareMap=hardwareMap;
        //fullyAutoMode=true;
        HeadingLockRadian = robotPosition.getData().headingRadian;
        noHeadModeStartError=robotPosition.getData().headingRadian;
    }
    /**
     * Auto初始化时调用
     * @param hardwareMap 硬件映射
     * @param initialPose 初始位置（roadrunner方向）
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
        if(!fullyAutoMode){
            if(runningToPoint){
                if (Math.abs(Math.hypot(vx,vy))>=PARAMS.zeroThresholdV||Math.abs(omega)>=PARAMS.zeroThresholdOmega) {
                    runningToPoint = false;//打断自动驾驶
                }else{
                    actionRunner.update();
                    if(!actionRunner.isBusy()){
                        runningToPoint = false;
                    }
//                    if(targetPoint==null){
//                        targetPoint=robotPosition.getData().getPosition(DistanceUnit.MM);
//                    }
//                    if(Double.isNaN(targetRadian)){
//                        if(HeadingLockRadianReset) {
//                            targetRadian = robotPosition.getData().headingRadian;
//                        }else{
//                            targetRadian=HeadingLockRadian;
//                        }
//                    }
//                    wheelSpeeds = chassisCalculator.solveGround(chassisCalculator.calculatePIDXY(targetPoint, robotPosition.getData().getPosition(DistanceUnit.MM)),
//                            chassisCalculator.calculatePIDRadian(targetRadian,robotPosition.getData().headingRadian),robotPosition.getData().headingRadian );
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
}
@Config
class ChassisCalculator {
    public static class Params {
        //todo 调整参数
        public double rb = 0.23; // rb 车轮中心到机器人中心的基本半径 (m)
        public double skP = 0.002;//speed k
        public double skI = 0;
        public double skD = 0.00025;
        public double rkP = 0.8;//radian k
        public double rkI = 1;
        public double rkD = 0.1;
        public double hkP = 0.7;//speedHeading k
        public double hkI = 0;
        public double hkD = 0.1;
        public double drkP=0.2;
        public double drkI=0;
        public double drkD=0.05;
        public double powerfulPIDRadianUseTimeMS = 1000;
    }
    public static Params PARAMS = new Params();
    long lastHeadingSetTimeMS=0;
    PIDController pidSpeed;
    PIDController pidSpeedHeading;
    PIDController pidRadianHeadLock;
    PIDController pidRadianDrive;

    ChassisCalculator() {
        // 私有构造函数，防止外部实例化
        pidSpeed = new PIDController(PARAMS.skP, PARAMS.skI, PARAMS.skD);
        pidSpeedHeading = new PIDController(PARAMS.hkP, PARAMS.hkI, PARAMS.hkD);
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

    long lastTimeXY = 0;
    boolean firstRunXY = true;
    double thisTimeHeadingRadian = 0;
    AngleMeanFilter meanFilter = new AngleMeanFilter(10);
    Point2D lastcurrent=new Point2D(0,0);
    public double[] calculatePIDXY(Point2D target, Point2D current) {
        double errorX = target.getX() - current.getX();
        double errorY = target.getY() - current.getY();
        double distance = Math.hypot(errorX, errorY);
        double angleToTarget = Math.atan2(errorY, errorX);
        if (firstRunXY) {
            firstRunXY = false;
            lastTimeXY = System.nanoTime();
            pidSpeed.reset();
            pidSpeedHeading.reset();
            lastcurrent = current;
            meanFilter.reset();
        }
        thisTimeHeadingRadian = meanFilter.filter(Point2D.translate(current,Point2D.rotate(lastcurrent,Math.PI)).getRadian());
        lastcurrent=current;
        pidSpeed.setPID(PARAMS.skP, PARAMS.skI, PARAMS.skD);
        pidSpeedHeading.setPID(PARAMS.hkP, PARAMS.hkI, PARAMS.hkD);
        double headingError = MathSolver.normalizeAngle(angleToTarget - thisTimeHeadingRadian);
        double v = pidSpeed.calculate(distance, 0, (System.nanoTime() - lastTimeXY) / 1e9);
        double heading = pidSpeedHeading.calculate(0,headingError,(System.nanoTime() - lastTimeXY) / 1e9);
        lastTimeXY = System.nanoTime();
        double vx = v * Math.cos(angleToTarget-heading);
        double vy = v * Math.sin(angleToTarget-heading);
        return new double[]{vx, vy};
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