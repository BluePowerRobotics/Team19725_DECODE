package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {
    public DcMotorEx shooterMotor;
    Telemetry telemetry;
    double current_time;
    double previous_time;
    double current_error;
    double previous_error;
    public static double degreePertick = 0;
    public static double k_p;
    public static double k_i = 0;
    public static double k_d;
    public static double i;
    public static double max_i = 10000;
    public double Isum = 0;
    public Shooter(HardwareMap hardwareMap, Telemetry telemetryrc){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        this.telemetry = telemetryrc;
    }

    /**
     *
     * 单位：°/s
     *
     */
    public boolean shoot(double targetSpeed){
        current_time = System.currentTimeMillis()/1000.0;
        // 处理角度关系
        current_error = targetSpeed - (shooterMotor.getVelocity(AngleUnit.DEGREES) / degreePertick);
        double P = k_p * current_error;

        Isum += k_i * (current_error * (current_time - previous_time));

        if(Isum > max_i){
            Isum = max_i;
        } else if(Isum < -max_i){
            Isum = -max_i;
        }

        double D = k_d * (current_error - previous_error) / (current_time - previous_time);

        double Power = P + Isum + D;
        Power = Range.clip(Power, -1, 1);
        shooterMotor.setVelocity(Power, AngleUnit.DEGREES);
        previous_error = current_error;
        previous_time = current_time;
        if(Math.abs(current_error) < 50){
            return true;
        } else {
            return false;
        }
    }


}
