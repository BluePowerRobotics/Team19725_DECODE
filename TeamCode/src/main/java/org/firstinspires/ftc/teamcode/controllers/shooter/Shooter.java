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
import org.firstinspires.ftc.teamcode.utility.PIDController;

//单个弹射飞轮的PID控制器
@Config
public class Shooter {
    public static int SpeedTolerance = 80;

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
    public static double MinPower = 0.2;
    public static double k_p = 0.0008;
    public static double k_i = 0.5;
    public static double k_d = 0.08;
    public static double max_i = 0.2;
    //这里的small指的是小三角发射，而不是更小的PID

    public static double MinPower_small = 0.4;
    public static double k_p_small = 0.0005;
    public static double k_d_small = 0.08;
    public static double k_i_small = 0.6;
    public static double max_i_small = 0.3;

    private PIDController pidController;

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
    }

    /**
     *两套PID 分别对应大，小三角
     */
    public boolean shoot(int targetSpeed){
        if(targetSpeed < 750){
            pidController.setPID(k_p,k_i,k_d);
            pidController.setMaxI(max_i);
        }
        else{
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




}
