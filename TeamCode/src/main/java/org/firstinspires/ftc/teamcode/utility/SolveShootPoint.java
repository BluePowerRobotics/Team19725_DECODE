package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;

public class SolveShootPoint {
    private static double genhao2 = Math.sqrt(2);
    public static Pose2d solveREDShootPoint(Pose2d robotPose, double R) {
        double x0 = robotPose.position.x;
        double y0 = robotPose.position.y;
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
        return new Pose2d(ansX, ansY, Math.atan((72 - ansY) / (72 - ansX)));
    }
    public static Pose2d solveBLUEShootPoint(Pose2d robotPose, double R) {
        double x0 = robotPose.position.x;
        double y0 = robotPose.position.y;
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
        return new Pose2d(ansX, ansY, (Math.atan(( 72 - ansY) / (-ansX - 72)) + Math.PI));
    }
}
