package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.locate.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
public class ChassisController {
    @Config
    public static class Params{
        //todo 调整参数
        public static double maxV=0.5; // 最大线速度 (m/s)
        public static double maxA=0.5; // 最大加速度 (m/s²)
        public static double maxOmega=Math.PI*3/4; // 最大角速度 (rad/s)
        public static double zeroThresholdV =0.05; // 速度零点阈值 (m/s)
        public static double zeroThresholdOmega =Math.toRadians(5); // 角速度零点阈值 (rad/s)
    }
    RobotPosition robotPosition;
    HardwareMap hardwareMap;
    boolean fullyAutoMode = false;
    boolean useNoHeadMode=false;
    boolean runningToPoint=false;
    ChassisCalculator chassisCalculator= new ChassisCalculator();
    ChassisOutputter chassisOutputter;

    /**
     * OpMode初始化时调用
     */
    public ChassisController(HardwareMap hardwareMap){
        robotPosition= RobotPosition.refresh(hardwareMap,robotPosition.initialPosition,robotPosition.initialHeadingRadian);
        this.hardwareMap=hardwareMap;
        chassisOutputter=new ChassisOutputter(hardwareMap);
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
        fullyAutoMode=true;
        chassisOutputter=new ChassisOutputter(hardwareMap);
    }
    public void gamepadInput(double vx,double vy,double omega){
        vx=vx*Params.maxV;
        vy=vy*Params.maxV;
        omega=omega*Params.maxOmega;
        if(!fullyAutoMode){
            if(runningToPoint){
                if (Math.hypot(vx, vy) > Params.zeroThresholdV && Math.abs(omega) > Params.zeroThresholdOmega) {
                    runningToPoint = false;//打断自动驾驶
                }else{
                    //todo 调用自动驾驶
                }
            }
            if(!runningToPoint) {
                double[] wheelSpeeds;
                if (useNoHeadMode)
                    wheelSpeeds = chassisCalculator.solveGround(vx, vy, omega, robotPosition.getData().headingRadian);
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
        public static double rb = 0.23; // rb 车轮中心到机器人中心的基本半径 (m)
        // rb 车轮中心到机器人中心的基本半径 (m)
        public static double pkP = 1;//point k
        public static double pkI = 0;
        public static double pkD = 0;
        public static double rkP = 1;//radian k
        public static double rkI = 0;
        public static double rkD = 0;
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
        double v_fl = vy - vx + (2 * Params.rb * omega);
        double v_bl = vy + vx + (2 * Params.rb * omega);
        double v_br = vy - vx - (2 * Params.rb * omega);
        double v_fr = vy + vx - (2 * Params.rb * omega);

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

    long lastTimeXY = 0;
    boolean firstRunXY = true;

    public double[] calculatePIDXY(Point2D target, Point2D current) {
        if (firstRunXY) {
            firstRunXY = false;
            lastTimeXY = System.nanoTime();
            pidPoint.reset();
        }
        double errorX = target.x - current.x;
        double errorY = target.y - current.y;
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
        static double mmPerTick = (wheelDiameter * Math.PI) / CPR; // 每个编码器脉冲对应的线性位移 (m)
        static double maxRpm = 312; // 电机最大转速 (RPM)
    }

    HardwareMap hardwareMap;
    DcMotorEx leftFront, rightFront, leftBack, rightBack;

    ChassisOutputter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * 设置各个车轮的线速度 (mm/s)
     *
     * @param v_fl 左前轮速度
     * @param v_fr 右前轮速度
     * @param v_bl 左后轮速度
     * @param v_br 右后轮速度
     */
    public void setWheelVelocities(double v_fl, double v_fr, double v_bl, double v_br) {
        v_fl = v_fl / Params.mmPerTick;
        v_fr = v_fr / Params.mmPerTick;
        v_bl = v_bl / Params.mmPerTick;
        v_br = v_br / Params.mmPerTick;
        if (Math.abs(v_fl) > Params.maxRpm / 60 * Params.CPR || Math.abs(v_fr) > Params.maxRpm / 60 * Params.CPR || Math.abs(v_bl) > Params.maxRpm / 60 * Params.CPR || Math.abs(v_br) > Params.maxRpm / 60 * Params.CPR) {
            double maxV = Math.max(Math.max(Math.abs(v_fl), Math.abs(v_fr)), Math.max(Math.abs(v_bl), Math.abs(v_br)));
            v_fl = v_fl / maxV * Params.maxRpm / 60 * Params.CPR;
            v_fr = v_fr / maxV * Params.maxRpm / 60 * Params.CPR;
            v_bl = v_bl / maxV * Params.maxRpm / 60 * Params.CPR;
            v_br = v_br / maxV * Params.maxRpm / 60 * Params.CPR;
        }
//        telemetry.addData("v_fl", v_fl);
//        telemetry.addData("v_fr", v_fr);
//        telemetry.addData("v_bl", v_bl);
//        telemetry.addData("v_br", v_br);
        leftFront.setVelocity(v_fl);
        rightFront.setVelocity(v_fr);
        leftBack.setVelocity(v_bl);
        rightBack.setVelocity(v_br);
    }

    public void setWheelVelocities(double[] velocities) {
        setWheelVelocities(velocities[0], velocities[1], velocities[2], velocities[3]);
    }
}