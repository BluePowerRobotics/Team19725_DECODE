package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUSensor {
    private IMU imu;
    private YawPitchRollAngles yawPitchRollAngles;
    private boolean isInited=false;
    IMU.Parameters parameters;
    public IMUSensor(HardwareMap hardwareMap, String DeviceName, RevHubOrientationOnRobot orientation){
        imu = hardwareMap.get(IMU.class, DeviceName); // "imu"需与手机配置中的名称一致
        parameters = new IMU.Parameters(orientation);
        isInited = imu.initialize(parameters);
        if(isInited) yawPitchRollAngles=imu.getRobotYawPitchRollAngles();
        else isInited = imu.initialize(parameters);
    }

    public YawPitchRollAngles getYawPitchRollAngles() {

        if(isInited) {
            yawPitchRollAngles=imu.getRobotYawPitchRollAngles();
            return yawPitchRollAngles;
        }else {
            isInited = imu.initialize(parameters);
            yawPitchRollAngles=imu.getRobotYawPitchRollAngles();
            return yawPitchRollAngles;
        }
    }
    public boolean ifInitiated(){
        return isInited;
    }
    public YawPitchRollAngles reset(){
        if(isInited) {
            imu.resetYaw();
            yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
            return yawPitchRollAngles;
        }else{
            isInited = imu.initialize(parameters);
            yawPitchRollAngles=imu.getRobotYawPitchRollAngles();
            return yawPitchRollAngles;
        }
    }
    public double getYaw(AngleUnit angleUnit){
        return getYawPitchRollAngles().getYaw(angleUnit);
    }
}
