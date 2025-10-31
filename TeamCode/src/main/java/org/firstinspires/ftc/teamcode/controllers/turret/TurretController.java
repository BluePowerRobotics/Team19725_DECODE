package org.firstinspires.ftc.teamcode.controllers.turret;

import org.firstinspires.ftc.teamcode.controllers.turret.model.TurretInfo;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point3D;

import java.util.ArrayList;
import java.util.List;

public class TurretController {
}
class TurretCalculator{
    public static class Param{
        public static double g=9.8; // 重力加速度 (m/s²)
        public static Point3D target = new Point3D(0,0,0); // 目标位置 (m)
        public static double turretHeight=0; // 炮台高度 (m)
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
        return solve(theta, x0, y0, h, vtx, vty);
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
    private List<TurretInfo> solve(double theta, double x0, double y0, double h,
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
}