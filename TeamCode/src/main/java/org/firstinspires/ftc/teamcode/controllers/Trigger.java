package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//控制弹射扳机 —— 一个180舵机
@Config

public class Trigger {
    Servo triggerServo;
    public double TRIGGER_OPEN_POS;
    public double TRIGGER_CLOSE_POS;
    public Trigger(HardwareMap hardwareMap, String name, double OPEN_POS, double CLOSE_POS){
        this.triggerServo = hardwareMap.get(Servo.class, name);
        this.triggerServo.setDirection(Servo.Direction.FORWARD);
        this.triggerServo.setPosition(TRIGGER_CLOSE_POS);
        this.TRIGGER_OPEN_POS = OPEN_POS;
        this.TRIGGER_CLOSE_POS = CLOSE_POS;
    }
    public void open(){
        this.triggerServo.setPosition(TRIGGER_OPEN_POS);
    }
    public void close() {
        this.triggerServo.setPosition(TRIGGER_CLOSE_POS);
    }
}
