package org.firstinspires.ftc.teamcode.controllers.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.shooter.DRL.Agent;
import org.firstinspires.ftc.teamcode.utility.PIDController;

//单个弹射飞轮的PID控制器
@Config
public class Shooter {
    public static int SpeedTolerance = 35;

    public static double BlockPower = -0.3;
    public DcMotorEx shooterMotor;
    TelemetryPacket packet = new TelemetryPacket();
    double[] speedBuffer = new double[10];
    Telemetry telemetry;
    double Power = 0;
    double current_speed = 0;
    double current_time;
    double previous_time;
    double current_encoder = 0;
    double previous_encoder = 0;
    double current_error;
    double previous_error;
    public static double degreePertick = 0;
    public static double MinPower = 0.1;
    public static double k_p = 0.01;
    public static double k_i = 0.2;
    public static double k_d = 0.05;
    public static double max_i = 1;
    //这里的small指的是小三角发射，而不是更小的PID

    public static double MinPower_small = 0.3;
    public static double k_p_small = 0.011;
    public static double k_d_small = 0.08;
    public static double k_i_small = 0.05;
    public static double max_i_small = 0.6;

    private PIDController pidController;

    private Agent drlAgent;
    private boolean drlModelLoaded = false;
    public static boolean useDRLModel = false;
    public static double turretAngle = Math.PI / 4;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetryrc, String motorName, boolean ifReverse){
        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(ifReverse)
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetryrc;
        pidController = new PIDController(k_p, k_i, k_d, max_i);
        previous_time = System.currentTimeMillis();
        
        loadDRLModel();
    }

    private void loadDRLModel() {
        try {
            drlAgent = Agent.load("/sdcard/FIRST/agent_DDPG.ser");
            drlModelLoaded = true;
            telemetry.addData("DRL Model", "Loaded successfully");
            telemetry.update();
        } catch (Exception e) {
            drlModelLoaded = false;
            telemetry.addData("DRL Model", "Load failed: " + e.getMessage());
            telemetry.update();
        }
    }

    public double[] calculateDRLSpeed(double robotVx, double robotVy, double targetRelX, double targetRelY) {
        if (!drlModelLoaded || !useDRLModel) {
            return new double[]{0, 0, 0};
        }

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

            telemetry.addData("DRL Input", String.format("v(%.2f, %.2f), target(%.2f, %.2f)", robotVx, robotVy, targetRelX, targetRelY));
            telemetry.addData("DRL Output", String.format("(%.2f, %.2f, %.2f)", launchVelocity[0], launchVelocity[1], launchVelocity[2]));
            telemetry.update();

            return launchVelocity;
        } catch (Exception e) {
            telemetry.addData("DRL Error", e.getMessage());
            telemetry.update();
            return new double[]{0, 0, 0};
        }
    }

    public int calculateLaunchSpeedFromVector(double[] launchVelocity) {
        // 计算发射速度的模长（假设是三维向量的模）
        double speed = Math.sqrt(
            launchVelocity[0] * launchVelocity[0] +
            launchVelocity[1] * launchVelocity[1] +
            launchVelocity[2] * launchVelocity[2]
        );
        
        // 转换为适合电机的速度值（度/秒）
        int motorSpeed = (int) Math.round(speed);
        motorSpeed = Math.max(600, Math.min(1000, motorSpeed));
        
        return motorSpeed;
    }

    /**
     *两套PID 分别对应大，小三角
     */
    public boolean shoot(int targetSpeed){
        if(targetSpeed < 750){
            InstanceTelemetry.getTelemetry().addData("current PID", "BIG");
            pidController.setPID(k_p,k_i,k_d);
            pidController.setMaxI(max_i);
        }
        else{
            InstanceTelemetry.getTelemetry().addData("current PID", "SMALL");
            pidController.setPID(k_p_small, k_i_small, k_d_small);
            pidController.setMaxI(max_i_small);
        }
        //如果是double，不可以 == 0，需要写abs < 0.0001
        if(targetSpeed == 0){
            shooterMotor.setPower(0);
            pidController.reset();
            previous_time = System.currentTimeMillis();
            return true;
        }
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        current_time = System.currentTimeMillis();
        current_encoder = shooterMotor.getCurrentPosition();
        current_speed = shooterMotor.getVelocity(AngleUnit.DEGREES);
        double dt = (current_time - previous_time);
        if (dt <= 0) dt = 1; // 防止除零
        Power = pidController.calculate(targetSpeed, current_speed, dt);
        //避免射球时出现负功率，导致震荡或电机损伤
        if(targetSpeed > 0 && targetSpeed < 750){
            Power = Range.clip(Power, MinPower, 1);
        }
        else if(targetSpeed > 0 && targetSpeed >=750){
            Power = Range.clip(Power, MinPower_small, 1);
        }
        shooterMotor.setPower(Power);
        previous_time = current_time;
        if(Math.abs(targetSpeed - current_speed) < SpeedTolerance){
            return true;
        } else {
            return false;
        }
    }
    public void block(){
        //telemetry.addData("blockPower", BlockPower);
        shooterMotor.setPower(BlockPower);
    }

    public double getCurrent_encoder(){
        return current_encoder;
    }
    public double getPower(){
        return Power;
    }
    public double getCurrent_speed(){
        return current_speed;
    }

    public boolean shootWithDRL(int targetSpeed, double robotVx, double robotVy, double targetRelX, double targetRelY) {
        int actualTargetSpeed = targetSpeed;
        
        if (useDRLModel && drlModelLoaded) {
            double[] launchVelocity = calculateDRLSpeed(robotVx, robotVy, targetRelX, targetRelY);
            int drlSpeed = calculateLaunchSpeedFromVector(launchVelocity);
            if (drlSpeed > 0) {
                actualTargetSpeed = drlSpeed;
            }
        }
        
        return shoot(actualTargetSpeed);
    }

    public void setTurretAngle(double angle) {
        turretAngle = angle;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public boolean isDRLModelLoaded() {
        return drlModelLoaded;
    }

    public void setUseDRLModel(boolean use) {
        useDRLModel = use;
        telemetry.addData("DRL Mode", use ? "Enabled" : "Disabled");
        telemetry.update();
    }
}
