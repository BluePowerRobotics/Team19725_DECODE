package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//控制弹射扳机 —— 一个360舵机
@Config

public class Trigger {
    Servo triggerServo;
    public static double TRIGGER_OPEN_VEL = 1;
    public static double TRIGGER_CLOSE_VEL = 0.5;
    public static double TRIGGER_EMERGENCYSTOP_VEL= 0;
    public Trigger(HardwareMap hardwareMap){
        this.triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        this.triggerServo.setDirection(Servo.Direction.FORWARD);
        this.triggerServo.setPosition(TRIGGER_CLOSE_VEL);
    }
    public void open(){
        this.triggerServo.setPosition(TRIGGER_OPEN_VEL);
    }
    public void close() {
        this.triggerServo.setPosition(TRIGGER_CLOSE_VEL);
    }
    public void emergencyStop(){
        this.triggerServo.setPosition(TRIGGER_EMERGENCYSTOP_VEL);
    }
}
