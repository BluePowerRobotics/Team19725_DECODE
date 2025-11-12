package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//控制弹射扳机 —— 一个180舵机 *左右各一个
@Config

public class Trigger {
    Servo triggerServo_LEFT;
    Servo triggerServo_RIGHT;
    public double TRIGGER_OPEN_POS = 0;
    public double TRIGGER_CLOSE_POS = 0.5;
    public Trigger(HardwareMap hardwareMap){
        this.triggerServo_LEFT = hardwareMap.get(Servo.class, "trigger_left");
        this.triggerServo_RIGHT = hardwareMap.get(Servo.class, "trigger_right");
        this.triggerServo_LEFT.setDirection(Servo.Direction.REVERSE);
        this.triggerServo_RIGHT.setDirection(Servo.Direction.FORWARD);
        this.triggerServo_LEFT.setPosition(TRIGGER_CLOSE_POS);
        this.triggerServo_RIGHT.setPosition(TRIGGER_CLOSE_POS);
    }
    public void open(){
        this.triggerServo_LEFT.setPosition(TRIGGER_OPEN_POS);
        this.triggerServo_RIGHT.setPosition(TRIGGER_OPEN_POS);
    }
    public void close() {
        this.triggerServo_LEFT.setPosition(TRIGGER_CLOSE_POS);
        this.triggerServo_RIGHT.setPosition(TRIGGER_CLOSE_POS);
    }
}
