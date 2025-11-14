package org.firstinspires.ftc.teamcode.controllers.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// 控制双发射轮的共同运作和自动化动作
@Config
public class ShooterAction {
    public static int speed_block = -400;
    public static int speed2_2 = 900;
    public static int speed25_25 = 925;
    public static int speed3_3 = 975;
    public static int speed25_55 = 1100;
    public static int speed35_55 = 1230;
    private Telemetry telemetry;
    private Shooter shooter_Left;
    private Shooter shooter_Right;

    public static int targetSpeed_low = 1600;
    public static int targetSpeed_high = 3000;
    public static int low_speed_threshold = 100;
    public ShooterAction(HardwareMap hardwareMap, Telemetry telerc) {
        shooter_Left = new Shooter(hardwareMap, telemetry, "shooterMotor1", true);
        shooter_Right = new Shooter(hardwareMap, telemetry, "shooterMotor2", true);
        telemetry = telerc;
    }
    public boolean setShootSpeed(int Power){
        boolean left = shooter_Left.shoot(Power);
        boolean right = shooter_Right.shoot(Power);
        return (left || right);
    }
    public void block(){
        shooter_Left.block();
        shooter_Right.block();
    }


    public class SpeedUp implements Action {
        private int targetSpeed;
        public SpeedUp(int targetSpeed) {
            this.targetSpeed = targetSpeed;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            boolean LeftisOK = shooter_Left.shoot(targetSpeed);
            boolean RightisOk = shooter_Right.shoot(targetSpeed);
            if(LeftisOK && RightisOk) {
                return false;
                // 达到目标速度，结束动作
            }
            return true;
        }
    }
    public Action SpeedUp(int targetSpeed) {
        return new SpeedUp(targetSpeed);
    }

    public class ShootThreeArtifacts implements Action {
        int targetSpeed;
        private int cnt = 0;
        boolean hasShot = false;

        Telemetry tele;
        //todo: delete tele;
        public ShootThreeArtifacts(int targetSpeed) {
            this.targetSpeed = targetSpeed;
        }


        @Override
        public boolean run(TelemetryPacket packet) {
            boolean LeftisOK = shooter_Left.shoot(targetSpeed);
            boolean RightisOk = shooter_Right.shoot(targetSpeed);
            if(!hasShot  &&  (shooter_Left.getCurrent_speed() < low_speed_threshold || shooter_Right.getCurrent_speed() < low_speed_threshold)) {
                cnt++; // 检测到球发射，动作结束
                hasShot = true;
            }
            if(LeftisOK && RightisOk) {
                hasShot = false;// 再次达到目标速度
            }
            if(cnt == 3){
                return false;
            }
            tele.addData("_____________CNT___________:", cnt);
            return true;
        }
    }
    public Action ShootThreeArtifacts(int targetSpeed) {
        return new ShootThreeArtifacts(targetSpeed);
    }


    public double getPower(){
        return shooter_Left.Power;
    }
    public double getCurrent_speed(){
        return shooter_Left.current_speed;
    }
}
