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
    public static double k_p = 0.005;
    //public static double k_p = 50;
    public static double k_i = 0;
    public static double k_d = 0;
    //public static double k_d = 0;
    public static double i;
    public static double max_i = 1 ;
    public double Isum = 0;
    public Shooter(HardwareMap hardwareMap, Telemetry telemetryrc, String motorName, boolean ifReverse){
        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(ifReverse)
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetryrc;
    }

    /**
     *
     * 单位：与°/s线性相关，但大概率不是°/s(idk)
     *应该比预期转速高80-100
     */
    //todo:fix low velocity issue
    public boolean shoot(double targetSpeed){
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        current_time = System.currentTimeMillis();
        current_encoder = shooterMotor.getCurrentPosition();
        current_speed = shooterMotor.getVelocity(AngleUnit.DEGREES);

        current_error = targetSpeed - current_speed;

        double P = k_p * current_error;

        Isum += k_i * (current_error * (current_time - previous_time));

        if(Isum > max_i){
            Isum = max_i;
        } else if(Isum < -max_i){
            Isum = -max_i;
        }

        double D = k_d * (current_error - previous_error) / (current_time - previous_time);

        Power = P + Isum + D;
        Power = Range.clip(Power, 0, 1);
        shooterMotor.setPower(Power);
        previous_error = current_error;
        previous_time = current_time;


//        telemetry.addData("power * 100", Power * 100);
//        telemetry.addData("degree", current_speed);
//        telemetry.addData("postion", shooterMotor.getCurrentPosition());
//        telemetry.addData("Position Vel", current_speed / 1000000000);
//        //telemetry.update();
//
//        packet.put("Shooter Power * 100", Power * 100);
//        packet.put("Shooter Speed", shooterMotor.getVelocity(AngleUnit.DEGREES));
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        if(Math.abs(current_error) < 50){
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
