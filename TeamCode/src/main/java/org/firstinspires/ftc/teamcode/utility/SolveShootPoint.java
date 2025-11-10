package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

//工具类，用于计算发射点位置
@Config
public class SolveShootPoint {
    public static double r1 = 48 * Math.sqrt(2);
    public static double r2 = 60 * Math.sqrt(2);
    public static double r3 = 72 * Math.sqrt(2);
    //特殊情况，表示小三角，靠近球门一边射击
    public static double r4 = 100;
    //特殊情况，表示小三角，远离球门一边射击
    public static double r5 = 200;
    private static double genhao2 = Math.sqrt(2);
    public static Pose2d solveREDShootPoint(Pose2d robotPose, double R) {
        Pose2d tmp = robotPose;
        Pose2d correct = new Pose2d(tmp.position.y,-tmp.position.x,0);


        //特殊判断小三角情况

        if(R == 100){
            return new Pose2d(-12, 60, -(Math.atan(5.0/11) + Math.PI/2));
        }
        if(R == 200){
            return new Pose2d(12, 60, -(Math.atan(7.0/11) + Math.PI/2));
        }

        double x0 = correct.position.x;
        double y0 = correct.position.y;
        double ansX = 0;
        double ansY = 0;

        //初始点在发射区内，找圆和车-球门连线的交点
        if(x0 < y0){
            double tmpx = 1 + Math.pow((72 - y0)/(72 - x0), 2);
            ansX = 72 - Math.sqrt(R*R / tmpx);
            ansY = 72 + (72 - y0)/(72 - x0) * (ansX - 72);
        }
        //初始点在发射区外，找圆和发射区边界的交点
        else{
            ansX = 72 - genhao2 / 2 * R;
            ansY = 72 - genhao2 / 2 * R;
        }
        return new Pose2d(ansY, -ansX, Math.atan((72 - ansY) / (72 - ansX)) + Math.PI / 2);
    }
    public static Pose2d solveBLUEShootPoint(Pose2d robotPose, double R) {
        Pose2d tmp = robotPose;
        Pose2d correct = new Pose2d(tmp.position.y,-tmp.position.x,0);

        if(R == 100){
            return new Pose2d(12, 60, -Math.atan(5.0/11));
        }
        if(R == 200){
            return new Pose2d(-12, 60, -Math.atan(7.0/11));
        }

        double x0 = correct.position.x;
        double y0 = correct.position.y;
        double ansX = 0;
        double ansY = 0;

        //初始点在发射区内，找圆和车-球门连线的交点
        if(-x0 < y0){
            double tmpx = 1 + Math.pow((72 - y0)/(-72 - x0), 2);
            ansX = -72 + Math.sqrt(R*R / tmpx);
            ansY = 72 + (72 - y0)/(-72 - x0) * (ansX +72);
        }
        //初始点在发射区外，找圆和发射区边界的交点
        else{
            ansX = -72 + genhao2 / 2 * R;
            ansY = 72 - genhao2 / 2 * R;
        }
        return new Pose2d(ansY, -ansX, (Math.atan(( 72 - ansY) / (-ansX - 72)) + 1.5* Math.PI));
    }
}
