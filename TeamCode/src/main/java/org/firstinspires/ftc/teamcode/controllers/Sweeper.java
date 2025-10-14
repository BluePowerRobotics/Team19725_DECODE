package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sweeper {
    public DcMotorEx motor = null;

    public Sweeper(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void Eat(double power){
        motor.setPower(power);
    }
    public void Eat(){
        motor.setPower(0.5);
    }
    public void GiveArtifact(){
        motor.setPower(0.3);
    }
    public void stop(){motor.setPower(0);}
    public double getPower(){
        return motor.getPower();
    }



}

