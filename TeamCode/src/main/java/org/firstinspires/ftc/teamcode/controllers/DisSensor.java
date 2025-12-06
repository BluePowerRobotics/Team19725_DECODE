package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DisSensor {
    public  DistanceSensor sensorDistance;
    public Rev2mDistanceSensor sensorTimeOfFlight;
    double t = 0;
    int frameCnt = 0;
    double sumDis = 0;

    double finalDis = 100000;//初始值


    public void init(HardwareMap hardwareMapRC) {
        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMapRC.get(DistanceSensor.class, "sensor_distance");

    }

    public double getDis() {
        return sensorDistance.getDistance(DistanceUnit.MM);
    } 
}
