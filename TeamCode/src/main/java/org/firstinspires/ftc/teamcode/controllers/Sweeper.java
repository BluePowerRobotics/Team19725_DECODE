package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Sweeper {
    public DcMotorEx motor = null;

    public static int EatVel = 1000;
    public static int GiveTheArtifactVel = 400;
    public static int OutputVel = -400;
    public Sweeper(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        //  motor.setDirection(DcMotor.Direction.REVERSE);
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
    public void stop(){motor.setVelocity(0);}
    public void output(){motor.setVelocity(OutputVel);}
    public double getPower(){
        return motor.getPower();
    }
    public double getVel(){return motor.getVelocity();}
    public double getCurrent(){return motor.getCurrent(CurrentUnit.AMPS);}




}

