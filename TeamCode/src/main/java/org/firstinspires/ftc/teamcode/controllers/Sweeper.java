package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Sweeper {
    public DcMotorEx motor = null;

    public static int EatVel = 2000;
    public static int GiveTheArtifactVel = 1000;
    public static int OutputVel = -1000;
    public Sweeper(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void Eat(double power){
        motor.setPower(power);
    }
    public void Eat(){
        motor.setVelocity(EatVel);
    }
    public void GiveArtifact(){
        motor.setPower(GiveTheArtifactVel);
    }
    public void stop(){motor.setPower(0);}
    public void output(){motor.setPower(-0.5);}
    public double getPower(){
        return motor.getPower();
    }
    public double getVel(){return motor.getVelocity();}



}

