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
    public static int speed2_2 = 700;
    public static int speed25_25 = 725;
    public static int speed3_3 = 750;
    public static int speed25_55 = 850;
    public static int speed35_55 = 875;
    private Telemetry telemetry;
    private Shooter shooter_Left;
    private Shooter shooter_Right;

    public static int targetSpeed_low = 700;
    public static int targetSpeed_high = 800;
    public static int speedDescend = 150;
    public int low_speed_threshold = 350;

    //第一个球被推入飞轮所需的时间
    public static long waitTime = 300;
    public static long ShootTime = 3300;
    public ShooterAction(HardwareMap hardwareMap, Telemetry telerc) {
        shooter_Left = new Shooter(hardwareMap, telemetry, "shooterMotor1", true);
        shooter_Right = new Shooter(hardwareMap, telemetry, "shooterMotor2", false);
        telemetry = telerc;
    }
    public boolean setShootSpeed(int Power){
        boolean left = shooter_Left.shoot(Power);
        boolean right = shooter_Right.shoot(Power);
        //todo 检查这里所有&&和||的逻辑
        return (left && right);
    }


    public class SpeedUp implements Action {
        private int targetSpeed;
        public SpeedUp(int targetSpeed) {
            this.targetSpeed = targetSpeed;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            telemetry.addData("Speed",shooter_Left.current_speed);
            telemetry.update();
            boolean LeftisOK = shooter_Left.shoot(targetSpeed);
            boolean RightisOk = shooter_Right.shoot(targetSpeed);
            if(LeftisOK || RightisOk) {
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
        boolean ifStart = false;
        boolean hasSetStopTime = false;
        long StartTime = 0;
        long StopTime = 0;
        int targetSpeed;
        private int cnt = 0;
        boolean hasShot = false;
        //todo: delete tele;
        public ShootThreeArtifacts(int targetSpeed) {
            this.targetSpeed = targetSpeed;
        }


        @Override
        public boolean run(TelemetryPacket packet) {
            if(!ifStart){
                low_speed_threshold = this.targetSpeed - speedDescend;
                StartTime = System.currentTimeMillis();
                ifStart = true;
            }
            telemetry.addData("Speed",shooter_Left.current_speed);
            telemetry.addData("cnt", cnt);
            telemetry.update();
            boolean LeftisOK = shooter_Left.shoot(targetSpeed);
            boolean RightisOk = shooter_Right.shoot(targetSpeed);
            //两个动作交接的时候，疑似产生不稳定因素使cnt在一开始就是1或2，所以在动作开始0.1s后再开始cnt的累加
            if( ( (System.currentTimeMillis() - StartTime) > waitTime) && !hasShot  &&  (shooter_Left.getCurrent_speed() < low_speed_threshold || shooter_Right.getCurrent_speed() < low_speed_threshold)) {
                cnt++; // 检测到球发射
                hasShot = true;
            }
            if(LeftisOK || RightisOk) {
                hasShot = false;// 再次达到目标速度
            }
            if(cnt >= 3){
                //如果立刻设置speed为0，会影响最后一个球的弹射，所以等0.1s再设置为0
                if(!hasSetStopTime){
                    StopTime = System.currentTimeMillis();
                    hasSetStopTime = true;
                }
                if( (System.currentTimeMillis() - StopTime) > 200){
                    shooter_Left.shoot(0);
                    shooter_Right.shoot(0);
                    return  false;
                }
            }
            if(System.currentTimeMillis() - StartTime > ShootTime){
                cnt += 10;
            }
            return true;
        }
    }
    public Action ShootThreeArtifacts(int targetSpeed) {
        return new ShootThreeArtifacts(targetSpeed);
    }


    public double getPower1(){
        return shooter_Left.Power;
    }
    public double getPower2(){
        return shooter_Right.Power;
    }

    public double getCurrent_speed1(){
        return shooter_Left.current_speed;
    }
    public double getCurrent_speed2(){
        return shooter_Right.current_speed;
    }
}
