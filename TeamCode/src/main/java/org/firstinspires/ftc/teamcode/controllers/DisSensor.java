package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
public class DisSensor {
    public  DistanceSensor sensorDistance;
    public Rev2mDistanceSensor sensorTimeOfFlight;


    public static double maxdistance=50;//此变量代表球满时与传感器的距离：最大值
    public static double wrongdistance=30;//那个b传感器，在3cm以内搁那乱转，神经病，写这个变量避免一下这种愚蠢行为。


    int frameCnt = 0;
    double sumDis = 0;

    double finalDis = 100000;//初始值


    /**
        *true 表示有3个球
     * false 表示3个球未满（未检测到球）
     */
    public boolean Whether_full(){
        double sensor_distance=sensorDistance.getDistance(DistanceUnit.MM);

        if (sensor_distance <= maxdistance & sensor_distance>=wrongdistance) {
            return true;
        }
        else{
            return false;
        }
    }

    public DisSensor(HardwareMap hardwareMap){
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }

    public double getDis() {
        return sensorDistance.getDistance(DistanceUnit.MM);
    } 
}
