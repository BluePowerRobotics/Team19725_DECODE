package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChassisOutputter {
    public static class Params {
        //todo 调整参数
        double CPR = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0); // 电机每转一圈的编码器脉冲数
        double wheelDiameter = 104; // 轮子直径 (mm)
        double mmPerTick = (wheelDiameter * Math.PI) / CPR; // 每个编码器脉冲对应的线性位移 (mm)
        double maxRpm = 312; // 电机最大转速 (RPM)
    }
    public static Params PARAMS = new Params();

    HardwareMap hardwareMap;
    DcMotorEx leftFront, rightFront, leftBack, rightBack;

    ChassisOutputter(HardwareMap hardwareMap) {
        //TODO 以下配置需要与MecanumDrive.java中保持一致
        this.hardwareMap = hardwareMap;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * 设置各个车轮的线速度 (m/s)
     *
     * @param v_fl 左前轮速度
     * @param v_fr 右前轮速度
     * @param v_bl 左后轮速度
     * @param v_br 右后轮速度
     */
    public void setWheelVelocities(double v_fl, double v_fr, double v_bl, double v_br) {
        v_fl = v_fl * 1000 / PARAMS.wheelDiameter;// (m/s) -> (r/s)
        v_fr = v_fr * 1000 / PARAMS.wheelDiameter;
        v_bl = v_bl * 1000 / PARAMS.wheelDiameter;
        v_br = v_br * 1000 / PARAMS.wheelDiameter;
        if(Math.abs(v_fl) > PARAMS.maxRpm / 60 || Math.abs(v_fr) > PARAMS.maxRpm / 60 || Math.abs(v_bl) > PARAMS.maxRpm / 60 || Math.abs(v_br) > PARAMS.maxRpm / 60){
            double maxV = Math.max(Math.max(Math.abs(v_fl), Math.abs(v_fr)), Math.max(Math.abs(v_bl), Math.abs(v_br)));
            v_fl = v_fl / maxV * PARAMS.maxRpm / 60;// range to [-maxRpm/60, maxRpm/60]
            v_fr = v_fr / maxV * PARAMS.maxRpm / 60;
            v_bl = v_bl / maxV * PARAMS.maxRpm / 60;
            v_br = v_br / maxV * PARAMS.maxRpm / 60;
        }
        leftFront.setPower(v_fl / (PARAMS.maxRpm / 60));
        rightFront.setPower(v_fr / (PARAMS.maxRpm / 60));
        leftBack.setPower(v_bl / (PARAMS.maxRpm / 60));
        rightBack.setPower(v_br / (PARAMS.maxRpm / 60));
    }

    public void setWheelVelocities(double[] velocities) {
        setWheelVelocities(velocities[0], velocities[1], velocities[2], velocities[3]);
    }
}
