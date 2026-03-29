package org.firstinspires.ftc.teamcode.utility.solvepoint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.shooter.DRL.Agent;

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

    public static double k = 50 / (r3 - r1);
    
    // DRL 相关变量
    private static Agent drlAgent;
    private static boolean drlModelLoaded = false;
    public static boolean useDRLModel = false;
    private static Telemetry telemetry;
    
    public static void initDRL(Telemetry telemetry) {
        SolveShootPoint.telemetry = telemetry;
        loadDRLModel();
    }
    
    private static void loadDRLModel() {
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
    
    public static void setUseDRLModel(boolean use) {
        useDRLModel = use;
        if (telemetry != null) {
            telemetry.addData("DRL Mode", use ? "Enabled" : "Disabled");
            telemetry.update();
        }
    }
    
    public static boolean isDRLModelLoaded() {
        return drlModelLoaded;
    }

    public static int solveShootSpeed(double distance){  
        return solveShootSpeed(distance, 0, 0, 0, 0);  
    }
    
    public static int solveShootSpeed(double distance, double robotVx, double robotVy, double targetRelX, double targetRelY){  
        if (useDRLModel && drlModelLoaded && drlAgent != null) {  
            return solveShootSpeedWithDRL(robotVx, robotVy, targetRelX, targetRelY);  
        } else {  
            // 传统方法  
            int speed = 0;  
            //大三角  
            if(distance < r3 + 10 * genhao2){  
                speed = Math.toIntExact(Math.round(1.081183337 * distance + 586.1640982));  
            }  
            //小三角  
            else{  
                speed = Math.toIntExact(Math.round(2.010562318 * distance + 540.4157073));  
            }  
            return speed;  
        }  
    }
    
    private static int solveShootSpeedWithDRL(double robotVx, double robotVy, double targetRelX, double targetRelY){  
        try {  
            double[] input = new double[4];  
            input[0] = robotVx;  // 机器人速度 x 分量  
            input[1] = robotVy;  // 机器人速度 y 分量  
            input[2] = targetRelX;  // 目标相对于机器人的 x 坐标  
            input[3] = targetRelY;  // 目标相对于机器人的 y 坐标  

            double[] action = drlAgent.decide(input);  
            double[] launchVelocity = new double[3];  
            
            // 确保输出是三维向量  
            if (action.length >= 3) {  
                launchVelocity[0] = action[0];  
                launchVelocity[1] = action[1];  
                launchVelocity[2] = action[2];  
            } else {  
                // 如果输出维度不足，使用默认值  
                launchVelocity[0] = 0;  
                launchVelocity[1] = 0;  
                launchVelocity[2] = 0;  
            }  

            // 计算速度向量的模长  
            double speed = Math.sqrt(  
                launchVelocity[0] * launchVelocity[0] +  
                launchVelocity[1] * launchVelocity[1] +  
                launchVelocity[2] * launchVelocity[2]  
            );  
            
            // 转换为适合电机的速度值（度/秒）  
            int motorSpeed = (int) Math.round(speed);  
            motorSpeed = Math.max(600, Math.min(1000, motorSpeed));  
            
            if (telemetry != null) {  
                telemetry.addData("DRL Input", String.format("v(%.2f, %.2f), target(%.2f, %.2f)", robotVx, robotVy, targetRelX, targetRelY));  
                telemetry.addData("DRL Output", String.format("(%.2f, %.2f, %.2f)", launchVelocity[0], launchVelocity[1], launchVelocity[2]));  
                telemetry.addData("Calculated Speed", motorSpeed);  
                telemetry.update();  
            }  

            return motorSpeed;  
        } catch (Exception e) {  
            if (telemetry != null) {  
                telemetry.addData("DRL Error", e.getMessage());  
                telemetry.update();  
            }  
            // 出错时使用传统方法  
            return solveShootSpeed(Math.sqrt(targetRelX * targetRelX + targetRelY * targetRelY));  
        }  
    }

    public static double solveBLUEShootHeading(Pose2d poseRC){
        double x = poseRC.position.x;
        double y = poseRC.position.y;
        double heading = Math.atan2(y+70.47,x+70.47);
        return heading;
    }
    public  static  double solveBLUEShootDistance(Pose2d poseRC){
        double x = poseRC.position.x;
        double y = poseRC.position.y;
        double distance = Math.sqrt((y+72)*(y+72) + (x+72) * (x+72));
        return distance;
    }


    public static Pose2d solveBLUEShootPoint(Pose2d poseRC, double r) {
        //特别判断小三角情况 100表示小三角离球门进近侧，200表示小三角离球门进远侧
        if(r == 100){
            return new Pose2d(60, -12, 0.4266274931);
        }
        if(r == 200){
            return new Pose2d(60, 12, 0.5636930906);
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
        }   //点在大三角区边线上
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
        double heading = Math.atan2(x+70.47,70.47-y) - Math.PI/2;
        return heading;
    }
    public  static  double solveREDShootDistance(Pose2d poseRC){
        double x = poseRC.position.x;
        double y = poseRC.position.y;
        double distance = Math.sqrt((y-72)*(y-72) + (x+72)*(x+72));
        return distance;
    }
    public static Pose2d solveREDShootPoint(Pose2d poseRC, double r) {

        if(r == 100){
            return new Pose2d(60, 12, -0.4266274931);
        }
        if(r == 200){
            return new Pose2d(60, -12, -0.5636930906);
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
