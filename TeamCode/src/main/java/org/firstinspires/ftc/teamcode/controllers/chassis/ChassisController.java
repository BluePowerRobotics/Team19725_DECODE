package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public static ChassisCalculator chassisCalculator= new ChassisCalculator();
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
                            chassisCalculator.pidRadian.reset();
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
@Config
class ChassisCalculator {
    public static class Params {
        //todo 调整参数
        public double rb = 0.23; // rb 车轮中心到机器人中心的基本半径 (m)
        public double skP = 0.002;//speed k
        public double skI = 0;
        public double skD = 0.00025;
        public double rkP = 0.7;//radian k
        public double rkI = 1.2;
        public double rkD = 0.1;
        public double hkP = 0.7;//speedHeading k
        public double hkI = 0;
        public double hkD = 0.1;
    }
    public static Params PARAMS = new Params();

    PIDController pidSpeed;
    PIDController pidSpeedHeading;
    PIDController pidRadian;

    ChassisCalculator() {
        // 私有构造函数，防止外部实例化
        pidSpeed = new PIDController(PARAMS.skP, PARAMS.skI, PARAMS.skD);
        pidSpeedHeading = new PIDController(PARAMS.hkP, PARAMS.hkI, PARAMS.hkD);
        pidRadian = new PIDController(PARAMS.rkP, PARAMS.rkI, PARAMS.rkD);
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
        double v_fl = vy + vx - (2 * PARAMS.rb * omega);
        double v_bl = vy - vx - (2 * PARAMS.rb * omega);
        double v_br = vy + vx + (2 * PARAMS.rb * omega);
        double v_fr = vy - vx + (2 * PARAMS.rb * omega);

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
            pidRadian.reset();
        }
        double errorRadian = targetRadian - currentRadian;
        // 归一化到[-π, π]
        errorRadian = MathSolver.normalizeAngle(errorRadian);
        pidRadian.setPID(PARAMS.rkP, PARAMS.rkI, PARAMS.rkD);
        double output = pidRadian.calculate(errorRadian, 0, (System.nanoTime() - lastTimeRadian) / 1e9);
        lastTimeRadian = System.nanoTime();
        return output;
    }
}
class ChassisOutputter {
    public static class Params {
        //todo 调整参数
        double CPR = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0); // 电机每转一圈的编码器脉冲数
        double wheelDiameter = 104; // 轮子直径 (mm)
        double mmPerTick = (wheelDiameter * Math.PI) / CPR; // 每个编码器脉冲对应的线性位移 (mm)
        double maxRpm = 312; // 电机最大转速 (RPM)
    }
    public static Params PARAMS = new Params();

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
        v_fl = v_fl * 1000 / PARAMS.wheelDiameter;// (m/s) -> (r/s)
        v_fr = v_fr * 1000 / PARAMS.wheelDiameter;
        v_bl = v_bl * 1000 / PARAMS.wheelDiameter;
        v_br = v_br * 1000 / PARAMS.wheelDiameter;
        if(Math.abs(v_fl) > PARAMS.maxRpm / 60 || Math.abs(v_fr) > PARAMS.maxRpm / 60 || Math.abs(v_bl) > PARAMS.maxRpm / 60 || Math.abs(v_br) > PARAMS.maxRpm / 60){
            double maxV = Math.max(Math.max(Math.abs(v_fl), Math.abs(v_fr)), Math.max(Math.abs(v_bl), Math.abs(v_br)));
            v_fl = v_fl / maxV * PARAMS.maxRpm / 60;// range to [-maxRpm/60, maxRpm/60]
            v_fr = v_fr / maxV * PARAMS.maxRpm / 60;
            v_bl = v_bl / maxV * PARAMS.maxRpm / 60;
            v_br = v_br / maxV * PARAMS.maxRpm / 60;
        }
        leftFront.setPower(v_fl / (PARAMS.maxRpm / 60));
        InstanceTelemetry.getTelemetry().addData("LF",v_fl / (PARAMS.maxRpm / 60));
        rightFront.setPower(v_fr / (PARAMS.maxRpm / 60));
        InstanceTelemetry.getTelemetry().addData("RF",v_fr / (PARAMS.maxRpm / 60));
        leftBack.setPower(v_bl / (PARAMS.maxRpm / 60));
        InstanceTelemetry.getTelemetry().addData("LB",v_bl / (PARAMS.maxRpm / 60));
        rightBack.setPower(v_br / (PARAMS.maxRpm / 60));
        InstanceTelemetry.getTelemetry().addData("RB",v_br / (PARAMS.maxRpm / 60));
    }

    public void setWheelVelocities(double[] velocities) {
        setWheelVelocities(velocities[0], velocities[1], velocities[2], velocities[3]);
    }
}