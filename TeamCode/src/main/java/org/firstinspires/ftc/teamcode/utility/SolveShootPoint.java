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

    public static double solveBLUEShootHeading(Pose2d poseRC){
        double x = poseRC.position.x;
        double y = poseRC.position.y;
        double heading = Math.atan((y+72)/(x+72));
        return heading;
    }

    public static Pose2d solveBLUEShootPoint(Pose2d poseRC, double r) {
        //特别判断小三角情况 100表示小三角离球门进近侧，200表示小三角离球门进远侧
        if(r == 100){
            return new Pose2d(45.084524053, -8.746427842, Math.atan(3.0/5.0));
        }
        if(r == 200){
            return new Pose2d(60, 12, Math.atan(84.0/(60+72)));
        }

        double x = poseRC.position.x;
        double y = poseRC.position.y;
        //最近点在大三角区内
        if (x < y) {
            double d = Math.hypot(x+72,y+72);
            double ansX = r*(x+72)/d-72;
            double ansY = r*(y+72)/d-72;
            double heading = Math.atan((y+72)/(x+72));
            Pose2d pose = new Pose2d(ansX, ansY, heading);
            return pose;
}       //点在大三角区边线上
        else {
            double ansX = r/Math.sqrt(2)-72;
            double ansY = r/Math.sqrt(2)-72;
            Pose2d pose = new Pose2d(ansX, ansY, Math.PI/4);
            return pose;
        }
    }

    public static double solveREDShootHeading(Pose2d poseRC){
        double x = poseRC.position.x;
        double y = poseRC.position.y;
        double heading = Math.atan((x+72)/(72-y)) - Math.PI/2;
        return heading;
    }
    public static Pose2d solveREDShootPoint(Pose2d poseRC, double r) {

        if(r == 100){
            return new Pose2d(45.084524053, 8.746427842, -Math.atan(3.0/5.0));
        }
        if(r == 200){
            return new Pose2d(60, -12, -Math.atan(84.0/(60+72)));
        }

        double x = poseRC.position.x;
        double y = poseRC.position.y;
        if (x+y < 0) {
            double d = Math.hypot(x+72,72-y);
            double ansX = r*(x+72)/d-72;
            double ansY = 72-r*(72-y)/d;
            double heading = Math.atan((x+72)/(72-y)) - Math.PI/2;
            Pose2d pose = new Pose2d(ansX, ansY, heading);
            return pose;
        }
        else {
            double ansX = r/Math.sqrt(2)-72;
            double ansY = 72-r/Math.sqrt(2);
            Pose2d pose = new Pose2d(ansX, ansY, -Math.PI/4);
            return pose;
        }
    }
}
