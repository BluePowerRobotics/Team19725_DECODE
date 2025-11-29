package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.Point2D;
@Config
public class ChassisCalculator {
    public static class Params {
        //todo 调整参数
        public double rb = 0.23; // rb 车轮中心到机器人中心的基本半径 (m)
        // rb 车轮中心到机器人中心的基本半径 (m)
        public double pkP = 0.002;//point k
        public double pkI = 0;
        public double pkD = 0.00025;
        public double rkP = 0.7;//radian k
        public double rkI = 0;
        public double rkD = 0.1;
    }
    public static Params PARAMS = new Params();

    PIDController pidPoint;
    PIDController pidRadian;

    ChassisCalculator() {
        // 私有构造函数，防止外部实例化
        pidPoint = new PIDController(PARAMS.pkP, PARAMS.pkI, PARAMS.pkD);
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
        pidPoint.setPID(PARAMS.pkP, PARAMS.pkI, PARAMS.pkD);
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
        errorRadian = MathSolver.normalizeAngle(errorRadian);
        pidRadian.setPID(PARAMS.rkP, PARAMS.rkI, PARAMS.rkD);
        double output = pidRadian.calculate(errorRadian, 0, (System.nanoTime() - lastTimeRadian) / 1e9);
        lastTimeRadian = System.nanoTime();
        return output;
    }
}
