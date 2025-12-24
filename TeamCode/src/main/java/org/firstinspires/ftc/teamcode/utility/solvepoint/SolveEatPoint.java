package org.firstinspires.ftc.teamcode.utility.solvepoint;

import com.acmerobotics.roadrunner.Pose2d;

public class SolveEatPoint {
    public static double solveBLUEEatHeading(Pose2d poseRC){
        double x = poseRC.position.x;
        double y = poseRC.position.y;
        double heading = Math.atan2(y-72, x-72) - Math.PI;
        return heading;
    }

    public static double solveREDEatHeading(Pose2d poseRC){
        double x = poseRC.position.x;
        double y = poseRC.position.y;
        double heading = Math.atan2(y+72, x-72) + Math.PI;
        return heading;
    }
}
