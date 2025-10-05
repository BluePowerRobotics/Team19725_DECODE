package org.firstinspires.ftc.teamcode.controllers.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterAction {
    private Telemetry telemetry;
    private Shooter shooter_Left;
    private Shooter shooter_Right;

    public static double targetSpeed_low = 1600;
    public static double targetSpeed_high = 3000;
    public static double low_speed_threshold = 100;
    public ShooterAction(HardwareMap hardwareMap, Telemetry telerc) {
        shooter_Left = new Shooter(hardwareMap, telemetry, "shooterMotor1", false);
        shooter_Right = new Shooter(hardwareMap, telemetry, "shooterMotor2", true);
        telemetry = telerc;
    }
    public class SpeedUp implements Action {
        private double targetSpeed;
        public SpeedUp(double targetSpeed) {
            this.targetSpeed = targetSpeed;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            shooter_Left.shoot(targetSpeed);
            shooter_Right.shoot(targetSpeed);
            if(Math.abs(shooter_Left.getCurrent_speed() - targetSpeed) <= 50 && Math.abs(shooter_Right.getCurrent_speed() - targetSpeed) <= 50) {
                return false;
                // 达到目标速度，结束动作
            }
            return true;
        }
    }
    public Action SpeedUp(double targetSpeed) {
        return new SpeedUp(targetSpeed);
    }

    public class ShootThreeArtifacts implements Action {
        double targetSpeed;
        private int cnt = 0;
        boolean hasShot = false;

        Telemetry tele;
        //todo: delete tele;
        public ShootThreeArtifacts(double targetSpeed) {
            this.targetSpeed = targetSpeed;
        }


        @Override
        public boolean run(TelemetryPacket packet) {
            shooter_Left.shoot(targetSpeed);
            shooter_Right.shoot(targetSpeed);
            if(!hasShot  &&  (shooter_Left.getCurrent_speed() < low_speed_threshold || shooter_Right.getCurrent_speed() < low_speed_threshold)) {
                cnt++; // 检测到球发射，动作结束
                hasShot = true;
            }
            if(Math.abs(shooter_Left.getCurrent_speed() - targetSpeed) <= 50 && Math.abs(shooter_Right.getCurrent_speed() - targetSpeed) <= 50) {
                hasShot = false;// 再次达到目标速度
            }
            if(cnt == 3){
                return false;
            }
            tele.addData("_____________CNT___________:", cnt);
            return true;
        }
    }
    public Action ShootThreeArtifacts(double targetSpeed) {
        return new ShootThreeArtifacts(targetSpeed);
    }
}
