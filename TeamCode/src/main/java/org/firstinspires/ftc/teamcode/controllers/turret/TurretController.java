package org.firstinspires.ftc.teamcode.controllers.turret;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.shooter.DRL.Agent;
import org.firstinspires.ftc.teamcode.controllers.turret.model.TurretInfo;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point3D;

import java.util.ArrayList;
import java.util.List;

public class TurretController {
    private Agent drlAgent;
    private boolean drlModelLoaded = false;
    private boolean useDRLModel = false;
    private Telemetry telemetry;

    public TurretController(Telemetry telemetry) {
        this.telemetry = telemetry;
        loadDRLModel();
    }

    public TurretController() {
        this.telemetry = null;
        loadDRLModel();
    }

    private void loadDRLModel() {
        try {
            drlAgent = Agent.load("/sdcard/FIRST/agent_DDPG.ser");
            drlModelLoaded = true;
            if (telemetry != null) {
                telemetry.addData("DRL Model", "Loaded successfully");
                telemetry.update();
            }
        } catch (Exception e) {
            drlModelLoaded = false;
            if (telemetry != null) {
                telemetry.addData("DRL Model", "Load failed: " + e.getMessage());
                telemetry.update();
            }
        }
    }

    public void setUseDRLModel(boolean use) {
        useDRLModel = use;
        if (telemetry != null) {
            telemetry.addData("DRL Mode", use ? "Enabled" : "Disabled");
            telemetry.update();
        }
    }

    public boolean isDRLModelLoaded() {
        return drlModelLoaded;
    }

    public boolean isUseDRLModel() {
        return useDRLModel;
    }

    public TurretCalculator createCalculator() {
        return new TurretCalculator(drlAgent, useDRLModel);
    }

    public TurretCalculator createCalculator(boolean useDRL) {
        return new TurretCalculator(drlAgent, useDRL);
    }
class TurretCalculator{
    private Agent drlAgent;
    private boolean useDRLModel;
    
    public TurretCalculator(Agent drlAgent, boolean useDRLModel) {
        this.drlAgent = drlAgent;
        this.useDRLModel = useDRLModel;
    }
    
    public void setDRLAgent(Agent drlAgent) {
        this.drlAgent = drlAgent;
    }
    
    public void setUseDRLModel(boolean use) {
        this.useDRLModel = use;
    }
    
    public static class Param{
        public static double g=9.8; // 重力加速度 (m/s²)
        public static Point3D target = new Point3D(0,0,0); // 目标位置 (m)
        public static double turretHeight=0; // 炮台高度 (m)
    }
    /**
     * 使用 DRL 模型解算炮台发射参数(固定仰角)
     * @param theta 炮台仰角 (rad)
     * @param x 炮台x位置
     * @param y 炮台y位置
     * @param vx 炮台x方向速度
     * @param vy 炮台y方向速度
     * @return List<TurretInfo> 所有可能解
     */
    public List<TurretInfo> solveSpeedWithDRL(double theta, double x, double y, double vx, double vy){
        if (!useDRLModel || drlAgent == null) {
            return solveSpeed(theta, x, y, vx, vy);
        }

        try {
            double x0 = Param.target.getX() - x;
            double y0 = Param.target.getY() - y;
            double h = Param.target.getZ() - Param.turretHeight;
            double vtx = -vx;
            double vty = -vy;

            double[] input = new double[4];
            input[0] = vx;  // 机器人速度 x 分量
            input[1] = vy;  // 机器人速度 y 分量
            input[2] = x0;  // 目标相对于机器人的 x 坐标
            input[3] = y0;  // 目标相对于机器人的 y 坐标

            double[] action = drlAgent.decide(input);
            List<TurretInfo> results = new ArrayList<>();

            if (action.length >= 3) {
                double v = Math.sqrt(action[0] * action[0] + action[1] * action[1] + action[2] * action[2]);
                double phi = Math.atan2(action[1], action[0]);
                
                double xt = x0 + vtx * 0.1;
                double yt = y0 + vty * 0.1;
                double phi_traditional = Math.atan2(yt, xt);

                results.add(new TurretInfo(v, phi_traditional, theta, 0.1));
            }

            return results;
        } catch (Exception e) {
            return solveSpeed(theta, x, y, vx, vy);
        }
    }

    /**
     * 使用 DRL 模型解算炮台发射参数(固定仰角)
     * @param theta 炮台仰角 (rad)
     * @param pose2d 炮台坐标
     * @param poseVelocity2d 炮台速度
     * @return List<TurretInfo> 所有可能解
     */
    public List<TurretInfo> solveSpeedWithDRL(double theta, Pose2d pose2d, PoseVelocity2d poseVelocity2d){
        return solveSpeedWithDRL(theta,-pose2d.position.y,+pose2d.position.x,-poseVelocity2d.linearVel.y,+poseVelocity2d.linearVel.x);
    }

    /**
     * 解算炮台发射参数(固定仰角)
     * @param theta 炮台仰角 (rad)
     * @param pose2d 炮台坐标
     * @param poseVelocity2d 炮台速度
     * @return List<TurretInfo> 所有可能解
     */
    public List<TurretInfo> solveSpeed(double theta, Pose2d pose2d, PoseVelocity2d poseVelocity2d){
        return solveSpeed(theta,-pose2d.position.y,+pose2d.position.x,-poseVelocity2d.linearVel.y,+poseVelocity2d.linearVel.x);
    }
    /**
     * 解算炮台发射参数(固定仰角)
     * @param theta 炮台仰角 (rad)
     * @param x 炮台x位置
     * @param y 炮台y位置
     * @param vx 炮台x方向速度
     * @param vy 炮台y方向速度
     * @return List<TurretInfo> 所有可能解
     */
    public List<TurretInfo> solveSpeed(double theta, double x, double y, double vx, double vy){
        double x0 = Param.target.getX() - x;
        double y0 = Param.target.getY() - y;
        double h = Param.target.getZ() - Param.turretHeight;
        double vtx = -vx;
        double vty = -vy;
        return solveSpeed(theta, x0, y0, h, vtx, vty);
    }
    /**
     * 解算炮台发射参数(固定仰角)
     * @param theta 炮台仰角 (rad)
     * @param x0 目标初始x位置差
     * @param y0 目标初始y位置差
     * @param h 炮台与目标的高度差 (目标高则为正)
     * @param vtx 目标x方向速度
     * @param vty 目标y方向速度
     * @return List<TurretInfo> 所有可能解
     */
    private List<TurretInfo> solveSpeed(double theta, double x0, double y0, double h,
                                        double vtx, double vty){
        double g = Param.g;
        List<TurretInfo> results = new ArrayList<>();

        double A = Math.cos(theta) / Math.sin(theta); // cot(theta)
        double R0_sq = x0 * x0 + y0 * y0;
        double d = x0 * vtx + y0 * vty;
        double vt_sq = vtx * vtx + vty * vty;

        // 四次方程系数
        double a4 = (A * A * g * g) / 4.0;
        double a3 = 0;
        double a2 = (A * A * h * g - vt_sq);
        double a1 = -2 * d;
        double a0 = (A * A * h * h - R0_sq);

        // 调用四次方程求解器
        double[] roots = MathSolver.solve4(a4, a3, a2, a1, a0);
        if(roots==null||roots.length==0){
            return results; // 无实数根
        }
        for (double t : roots) {
            if (t <= 0) continue; // 时间必须为正

            double v = (h + 0.5 * g * t * t) / (t * Math.sin(theta));
            double xt = x0 + vtx * t;
            double yt = y0 + vty * t;
            double phi = Math.atan2(yt, xt);

            results.add(new TurretInfo(v, phi, theta, t));
        }

        return results;
    }
    /**
     * 解算炮台发射参数(固定初速度)
     * @param speed 发射初速度 (m/s)
     * @param x0 目标初始x位置差 (m)
     * @param y0 目标初始y位置差 (m)
     * @param h 炮台与目标的高度差 (m，目标高则为正)
     * @param vtx 目标x方向速度 (m/s)
     * @param vty 目标y方向速度 (m/s)
     * @return List<TurretInfo> 所有可能解
     */
    public List<TurretInfo> solveTheta(double speed, double x0, double y0, double h,
                                       double vtx, double vty) {
        double g = Param.g;
        List<TurretInfo> results = new ArrayList<>();

        // 计算四次方程系数
        double a4 = g * g / 4.0;
        double a3 = 0.0;
        double a2 = vtx * vtx + vty * vty - speed * speed + g * h;
        double a1 = 2 * (x0 * vtx + y0 * vty);
        double a0 = x0 * x0 + y0 * y0 + h * h;

        // 求解四次方程
        double[] roots = MathSolver.solve4(a4, a3, a2, a1, a0);
        if (roots == null || roots.length == 0) {
            return results; // 无实数根
        }

        for (double t : roots) {
            if (t <= 0) continue; // 时间必须为正

            // 计算速度分量
            double Vx = x0 / t + vtx;
            double Vy = y0 / t + vty;
            double Vz = h / t + 0.5 * g * t;

            // 计算角度
            double phi = Math.atan2(Vy, Vx);
            double theta = Math.atan2(Vz, Math.sqrt(Vx * Vx + Vy * Vy));

            results.add(new TurretInfo(speed, phi, theta, t));
        }

        return results;
    }

    /**
     * 解算炮台发射参数(固定初速度)
     * @param speed 发射初速度 (m/s)
     * @param pose2d 炮台坐标
     * @param poseVelocity2d 炮台速度
     * @return List<TurretInfo> 所有可能解
     */
    public List<TurretInfo> solveTheta(double speed, Pose2d pose2d, PoseVelocity2d poseVelocity2d) {
        return solveTheta(speed, -pose2d.position.y, +pose2d.position.x,
                -poseVelocity2d.linearVel.y, +poseVelocity2d.linearVel.x);
    }

    /**
     * 解算炮台发射参数(固定初速度)
     * @param speed 发射初速度 (m/s)
     * @param x 炮台x位置 (m)
     * @param y 炮台y位置 (m)
     * @param vx 炮台x方向速度 (m/s)
     * @param vy 炮台y方向速度 (m/s)
     * @return List<TurretInfo> 所有可能解
     */
    public List<TurretInfo> solveTheta(double speed, double x, double y, double vx, double vy) {
        double x0 = Param.target.getX() - x;
        double y0 = Param.target.getY() - y;
        double h = Param.target.getZ() - Param.turretHeight;
        double vtx = -vx;
        double vty = -vy;

        return solveTheta(speed, x0, y0, h, vtx, vty);
    }
}