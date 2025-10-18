package org.firstinspires.ftc.teamcode.controllers.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public static double k_p = 0.017;
    public static double k_i = 0;
    public static double k_d = 0.05;
    public static double max_i = 1;
    private PIDController pidController;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetryrc, String motorName, boolean ifReverse){
        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
     *
     * 单位：与°/s线性相关，但大概率不是°/s(idk)
     *应该比预期转速高80-100
     */
    //todo:fix low velocity issue
    public boolean shoot(double targetSpeed){
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
        Power = Range.clip(Power, 0.001, 1);
        shooterMotor.setPower(Power);
        previous_time = current_time;
        if(Math.abs(targetSpeed - current_speed) < 50){
            return true;
        } else {
            return false;
        }
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
