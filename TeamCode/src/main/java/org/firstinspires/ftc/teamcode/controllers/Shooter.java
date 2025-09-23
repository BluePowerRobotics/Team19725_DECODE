package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {
    public DcMotorEx shooterMotor;
    TelemetryPacket packet = new TelemetryPacket();

    Telemetry telemetry;
    double current_time;
    double previous_time;
    double current_error;
    double previous_error;
    public static double degreePertick = 0;
    public static double k_p = 0.01;
    public static double k_i = 0;
    public static double k_d = 0.007;
    public static double i;
    public static double max_i = 1 ;
    public double Isum = 0;
    public Shooter(HardwareMap hardwareMap, Telemetry telemetryrc){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        this.telemetry = telemetryrc;
    }

    /**
     *
     * 单位：与°/s线性相关，但大概率不是°/s(idk)
     *应该比预期转速高80-100
     */
    //todo:fix low velocity issue
    public boolean shoot(double targetSpeed){
        current_time = System.currentTimeMillis();


        current_error = targetSpeed - (shooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("CurrentError", current_error);
        telemetry.addLine();
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
        shooterMotor.setPower(Power);
        previous_error = current_error;
        previous_time = current_time;


        telemetry.addData("power", Power);
        telemetry.addData("degree", shooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("postion", shooterMotor.getCurrentPosition());
        telemetry.addData("Position Vel", current_error / ((System.currentTimeMillis() - previous_time) / 1000));
        //telemetry.update();

        packet.put("Shooter Power", Power);
        packet.put("Shooter Speed", shooterMotor.getVelocity(AngleUnit.DEGREES));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        if(Math.abs(current_error) < 50){
            return true;
        } else {
            return false;
        }
    }


}
