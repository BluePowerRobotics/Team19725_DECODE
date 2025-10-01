package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sweeper {
    public DcMotorEx motor = null;

    public Sweeper(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void Eat(double power){
        motor.setPower(power);
    }
    public void Eat_fullpower(){
        motor.setPower(1);
    }
    public void Eat_halfpower(){
        motor.setPower(0.5);
    }
    public double getPower(){
        return motor.getPower();
    }



}
