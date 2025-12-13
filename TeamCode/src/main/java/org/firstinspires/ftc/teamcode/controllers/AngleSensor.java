package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.MathSolver;

public class AngleSensor {
    double ZeroDegreeVoltage=0;
    public AngleSensor(HardwareMap hardwareMap,String deviceName,double ZeroDegreeVoltage){
        analogInput = hardwareMap.get(AnalogInput.class,deviceName);
        this.ZeroDegreeVoltage=ZeroDegreeVoltage;
    }
    AnalogInput analogInput;
    public double getVoltage(){
        return analogInput.getMaxVoltage();
    }
    public double getRadian(){
        return MathSolver.normalizeAngle((getVoltage()-ZeroDegreeVoltage)/3.3*Math.PI*2);
    }
}
