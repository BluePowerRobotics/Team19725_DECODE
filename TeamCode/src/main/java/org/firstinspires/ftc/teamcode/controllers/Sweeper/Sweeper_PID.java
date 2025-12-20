package org.firstinspires.ftc.teamcode.controllers.Sweeper;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.PIDController;

//单个弹射飞轮的PID控制器
@Config
public class Sweeper_PID {
    private boolean ifReverse;
    public static double kf = 0.3;
    public static double minuskf = -0.3;
    public static int OutputSpeed = -400;
    public static int EatVel = 1960;
    public static int GiveTheArtifactVel = 1000;
    public DcMotorEx SweeperMotor;
    TelemetryPacket packet = new TelemetryPacket();
    Telemetry telemetry;
    double Power = 0;
    double current_speed = 0;
    double current_time;
    double previous_time;
    double current_encoder = 0;
    public static double minus_k_p = 0.1;
    public static double minus_k_i = 1.0;
    public static double minus_k_d = 0.000;
    public static double k_p_f = 0.1;
    public static double k_d_f = 0;
    public static double k_i_f = 1;
    public static double k_p = 0.1;
    public static double k_i = 1.0;
    public static double k_d = 0.000;
    public static double max_i = 1;
    public static double backRequireCycle=0.4;
    private PIDController pidController;

    public Sweeper_PID(HardwareMap hardwareMap, Telemetry telemetryrc, String motorName, boolean ifReverseRC) {
        this.ifReverse = ifReverseRC;
        SweeperMotor = hardwareMap.get(DcMotorEx.class, motorName);
        SweeperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (this.ifReverse)
            SweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            SweeperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetryrc;
        pidController = new PIDController(k_p, k_i, k_d, max_i);
        previous_time = System.currentTimeMillis();
    }

    /**
     *
     * 单位：与°/s线性相关，但大概率不是°/s(idk)
     * 应该比预期转速高80-100
     */
    //todo:fix low velocity issue
    public boolean Sweep(int targetSpeed) {
        if(targetSpeed > GiveTheArtifactVel){
            pidController.setPID(k_p, k_i, k_d);
        }
        else if(targetSpeed > 0){
            pidController.setPID(k_p_f, k_i_f, k_d_f);
        }
        else{
            pidController.setPID(minus_k_p, minus_k_i, minus_k_d);
        }
        //如果是double，不可以 == 0，需要写abs < 0.0001
        if (targetSpeed == 0) {
            SweeperMotor.setPower(0);
            pidController.reset();
            previous_time = System.currentTimeMillis();
            return true;
        }
        SweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        current_time = System.currentTimeMillis();
        current_encoder = SweeperMotor.getCurrentPosition();
        current_speed = SweeperMotor.getVelocity(AngleUnit.DEGREES);
        double dt = (current_time - previous_time);
        if (dt <= 0) dt = 1; // 防止除零
        Power = pidController.calculate(targetSpeed, current_speed, dt);
        //避免出现负功率，导致震荡或电机损伤
        if (targetSpeed > 0) {
            Power = Range.clip(Power, kf, 1);
        }
        else {
            Power = Range.clip(Power, -1, minuskf);
        }
        SweeperMotor.setPower(Power);
        previous_time = current_time;
        return Math.abs(targetSpeed - current_speed) < 50;
    }
    public class SweeperBack implements Action {
        private int sweeperBackStartTick = 0;
        private boolean hasSetStartTicks = false;
        @Override
        public boolean run(TelemetryPacket packet) {
            if(!hasSetStartTicks){
                sweeperBackStartTick = SweeperMotor.getCurrentPosition();
                hasSetStartTicks = true;
            }
            Sweep(OutputSpeed);
            if(SweeperMotor.getCurrentPosition() <= sweeperBackStartTick - Sweeper.tickPerCycle * backRequireCycle){
                SweeperMotor.setPower(0);
                return false;
            }
            return true;
        }
    }
    public Action SweeperBack() {
        return new SweeperBack();
    }

    public void output(){
        Sweep(OutputSpeed);
    }

    public class SweeperAction implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            Sweep(EatVel);
            return true;
        }
    }

    public Action SweeperAction() {
        return new SweeperAction();
    }


    public double getCurrent_encoder(){
        return current_encoder;
    }
    public double getPower(){
        return Power;
    }
    public double getCurrent_speed(){
        return current_speed;
    }




}
