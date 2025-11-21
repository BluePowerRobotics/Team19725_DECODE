package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
public class Sweeper {
    public DcMotorEx motor = null;
    public static double TimeThreshold = 300;
    public static double CurrentThreshold = 99;
    private long overStartTime = -1;
    public static int EatVel = 1960;
    public static int GiveTheArtifactVel = 1600;
    public static int OutputVel = -960;
    public Sweeper(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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


    public boolean isStuck(){
        double current = motor.getCurrent(CurrentUnit.AMPS);
        long now = System.currentTimeMillis();
        if (current > CurrentThreshold) {
            if (overStartTime < 0) {
                overStartTime = now;
            }
            return (now - overStartTime) > TimeThreshold;
        } else {
            overStartTime = -1;
            return false;
        }
    }
}

