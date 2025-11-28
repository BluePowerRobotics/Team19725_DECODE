package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        public static double maxV=0.5; // 最大线速度 (m/s)
        public static double maxA=0.5; // 最大加速度 (m/s²)
        public static double maxOmega=Math.PI*1/2; // 最大角速度 (rad/s)
        public static double zeroThresholdV =0.05; // 速度零点阈值 (m/s)
        public static double zeroThresholdOmega =Math.toRadians(5); // 角速度零点阈值 (rad/s)
    }
    public RobotPosition robotPosition;
    HardwareMap hardwareMap;
    boolean fullyAutoMode = false;
    public boolean useNoHeadMode=false;
    public boolean runningToPoint=false;
    boolean autoLockHeading=true;
    boolean HeadingLockRadianReset=true;
    double HeadingLockRadian;
    public double noHeadModeStartError;
    public ActionRunner actionRunner=new ActionRunner();
    ChassisCalculator chassisCalculator= new ChassisCalculator();
    ChassisOutputter chassisOutputter;

    /**
     * OpMode初始化时调用
     */
    public ChassisController(HardwareMap hardwareMap){
        robotPosition= RobotPosition.refresh(hardwareMap);
        this.hardwareMap=hardwareMap;
        chassisOutputter=new ChassisOutputter(hardwareMap);
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
        chassisOutputter=new ChassisOutputter(hardwareMap);
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
        this.HeadingLockRadian = MathSolver.normalizeAngle(headingLockRadian);
    }

    public void setTargetPoint(Pose2d pose2d){
        runningToPoint = true;
        TrajectoryActionBuilder actionBuilder = robotPosition.mecanumDrive.actionBuilder(robotPosition.getData().getPose2d())
                .strafeToLinearHeading(pose2d.position,pose2d.heading);
        actionRunner.clear();
        actionRunner.add(actionBuilder.build());
        HeadingLockRadian = pose2d.heading.log();
    }

    public void resetPosition(Pose2d pose2d){
        robotPosition=RobotPosition.refresh(hardwareMap,pose2d);
    }

    public double[] wheelSpeeds={0,0,0,0};
    public void gamepadInput(double vx,double vy,double omega){
        vx=vx*Params.maxV;
        vy=vy*Params.maxV;
        omega=omega*Params.maxOmega;
        if(!fullyAutoMode){
            if(runningToPoint){
                if (Math.abs(Math.hypot(vx,vy))>=Params.zeroThresholdV||Math.abs(omega)>=Params.zeroThresholdOmega) {
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
                            HeadingLockRadian=robotPosition.getData().headingRadian;
                        }
                        omega=chassisCalculator.calculatePIDRadian(HeadingLockRadian,robotPosition.getData().headingRadian);
                    }
                }
                if (useNoHeadMode)
                    wheelSpeeds = chassisCalculator.solveGround(vx, vy, omega, robotPosition.getData().headingRadian-noHeadModeStartError);
                else
                    wheelSpeeds = chassisCalculator.solveChassis(vx, vy, omega);
                chassisOutputter.setWheelVelocities(wheelSpeeds);
            }
        }
    }
}

class ChassisCalculator {
    @Config
    public static class Params {
        //todo 调整参数
        public static double rb = 0.23; // rb 车轮中心到机器人中心的基本半径 (m)
        // rb 车轮中心到机器人中心的基本半径 (m)
        public static double pkP = 0.002;//point k
        public static double pkI = 0;
        public static double pkD = 0.00025;
        public static double rkP = 0.7;//radian k
        public static double rkI = 0;
        public static double rkD = 0.1;
    }

    PIDController pidPoint;
    PIDController pidRadian;

    ChassisCalculator() {
        // 私有构造函数，防止外部实例化
        pidPoint = new PIDController(Params.pkP, Params.pkI, Params.pkD);
        pidRadian = new PIDController(Params.rkP, Params.rkI, Params.rkD);
    }

    /**
     * 逆运动学公式
     *
     * @param vx    机器人相对于自身的横移速度 (m/s) —— +右
     * @param vy    机器人相对于自身的前进速度 (m/s) —— +前
     * @param omega 机器人旋转角速度 (rad/s) —— +逆时针
     * @return 各个车轮的线速度 (m/s)
     */
    public double[] solveChassis(double vx, double vy, double omega) {
        double v_fl = vy + vx - (2 * Params.rb * omega);
        double v_bl = vy - vx - (2 * Params.rb * omega);
        double v_br = vy + vx + (2 * Params.rb * omega);
        double v_fr = vy - vx + (2 * Params.rb * omega);

        return new double[]{v_fl, v_fr, v_bl, v_br};
    }

    /**
     * 逆运动学公式（地面坐标系）
     * @param vx 机器人相对于地面的横移速度 (m/s) —— +右
     * @param vy 机器人相对于地面的前进速度 (m/s) —— +前
     * @param omega 机器人旋转角速度 (rad/s) —— +逆时针
     * @param headingRadian 机器人朝向，弧度制
     * @return 各个车轮的线速度 (m/s)
     */
    public double[] solveGround(double vx, double vy, double omega, double headingRadian) {

        double vxPro = vx * Math.cos(headingRadian) + vy * Math.sin(headingRadian);
        double vyPro = -vx * Math.sin(headingRadian) + vy * Math.cos(headingRadian);
        return solveChassis(vxPro, vyPro, omega);
    }
    public double[] solveGround(double[] vxy,double vOmega,double headingRadian){
        return solveGround(vxy[0],vxy[1],vOmega,headingRadian);
    }

    long lastTimeXY = 0;
    boolean firstRunXY = true;

    public double[] calculatePIDXY(Point2D target, Point2D current) {
        if (firstRunXY) {
            firstRunXY = false;
            lastTimeXY = System.nanoTime();
            pidPoint.reset();
        }
        double errorX = target.getX() - current.getX();
        double errorY = target.getY() - current.getY();
        double distance = Math.hypot(errorX, errorY);
        double angleToTarget = Math.atan2(errorY, errorX);
        pidPoint.setPID(Params.pkP, Params.pkI, Params.pkD);
        double output = pidPoint.calculate(distance, 0, (System.nanoTime() - lastTimeXY) / 1e9);
        lastTimeXY = System.nanoTime();
        double vx = output * Math.cos(angleToTarget);
        double vy = output * Math.sin(angleToTarget);
        return new double[]{vx, vy};
    }

    long lastTimeRadian = 0;
    boolean firstRunRadian = true;

    public double calculatePIDRadian(double targetRadian, double currentRadian) {
        if (firstRunRadian) {
            firstRunRadian = false;
            lastTimeRadian = System.nanoTime();
            pidRadian.reset();
        }
        double errorRadian = targetRadian - currentRadian;
        // 归一化到[-π, π]
        errorRadian = (errorRadian + Math.PI) % (2 * Math.PI) - Math.PI;
        pidRadian.setPID(Params.rkP, Params.rkI, Params.rkD);
        double output = pidRadian.calculate(errorRadian, 0, (System.nanoTime() - lastTimeRadian) / 1e9);
        lastTimeRadian = System.nanoTime();
        return output;
    }
}
class ChassisOutputter {
    public static class Params {
        //todo 调整参数
        static double CPR = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0); // 电机每转一圈的编码器脉冲数
        static double wheelDiameter = 104; // 轮子直径 (mm)
        static double mmPerTick = (wheelDiameter * Math.PI) / CPR; // 每个编码器脉冲对应的线性位移 (mm)
        static double maxRpm = 312; // 电机最大转速 (RPM)
    }

    HardwareMap hardwareMap;
    DcMotorEx leftFront, rightFront, leftBack, rightBack;

    ChassisOutputter(HardwareMap hardwareMap) {
        //TODO 以下配置需要与MecanumDrive.java中保持一致
        this.hardwareMap = hardwareMap;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * 设置各个车轮的线速度 (m/s)
     *
     * @param v_fl 左前轮速度
     * @param v_fr 右前轮速度
     * @param v_bl 左后轮速度
     * @param v_br 右后轮速度
     */
    public void setWheelVelocities(double v_fl, double v_fr, double v_bl, double v_br) {
        v_fl = v_fl * 1000 / Params.wheelDiameter;// (m/s) -> (r/s)
        v_fr = v_fr * 1000 / Params.wheelDiameter;
        v_bl = v_bl * 1000 / Params.wheelDiameter;
        v_br = v_br * 1000 / Params.wheelDiameter;
        if(Math.abs(v_fl) > Params.maxRpm / 60 || Math.abs(v_fr) > Params.maxRpm / 60 || Math.abs(v_bl) > Params.maxRpm / 60 || Math.abs(v_br) > Params.maxRpm / 60){
            double maxV = Math.max(Math.max(Math.abs(v_fl), Math.abs(v_fr)), Math.max(Math.abs(v_bl), Math.abs(v_br)));
            v_fl = v_fl / maxV * Params.maxRpm / 60;// range to [-maxRpm/60, maxRpm/60]
            v_fr = v_fr / maxV * Params.maxRpm / 60;
            v_bl = v_bl / maxV * Params.maxRpm / 60;
            v_br = v_br / maxV * Params.maxRpm / 60;
        }
        leftFront.setPower(v_fl / (Params.maxRpm / 60));
        rightFront.setPower(v_fr / (Params.maxRpm / 60));
        leftBack.setPower(v_bl / (Params.maxRpm / 60));
        rightBack.setPower(v_br / (Params.maxRpm / 60));
    }

    public void setWheelVelocities(double[] velocities) {
        setWheelVelocities(velocities[0], velocities[1], velocities[2], velocities[3]);
    }
}