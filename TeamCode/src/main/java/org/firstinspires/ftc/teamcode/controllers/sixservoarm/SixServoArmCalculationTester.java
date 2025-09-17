package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;
import org.firstinspires.ftc.teamcode.controllers.Point3D;

import org.firstinspires.ftc.teamcode.Vision.FindCandidate;
import org.firstinspires.ftc.teamcode.controllers.Shooter;

import java.util.Arrays;

@Config
@TeleOp(name = "918TeamDisplay", group = "Show")
public class SixServoArmCalculationTester extends LinearOpMode {
    //FindCandidate findCandidate = new FindCandidate();
    static Point3D SearchPoint = new Point3D(0, 220, 70);
    public static double deltaX = 15;//夹子到手臂中心的距离
    public static double deltaY = 0;
    SixServoArmController sixServoArmController;
    SixServoArmOutputter sixServoArmOutputter;
    SixServoArmState sixServoArmState;
    FindCandidate Cv = new FindCandidate();
    Shooter shooter = null;
    public static double initSpeed = 380;
    double targetSpeed = 0;
    public void initiate(){
        shooter = new Shooter(hardwareMap, telemetry);
        Cv.init(hardwareMap, telemetry, 0);
        SixServoArmController.setInstance(hardwareMap,telemetry);
        sixServoArmController=SixServoArmController.getInstance();
        sixServoArmOutputter = sixServoArmController.getOutputter();
        sixServoArmState = sixServoArmController.getState();
        sixServoArmController.setTargetPosition(new Point3D(0, 100, 30), (-Math.PI / 2), 0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initiate();
        waitForStart();
        last_time_ms=now_time_ms=System.currentTimeMillis();
        while (opModeIsActive()){
            addTele();
            if(!TestMode)
                setArm();
            else
                directServoControl();
            if(gamepad1.xWasReleased()){
                TestMode=!TestMode;
            }
            shoot();
        }
    }
    boolean TestMode=true;
    public void addTele(){
        last_time_ms=now_time_ms;
        now_time_ms=System.currentTimeMillis();
        telemetry.addData("FPS",1000.0/(now_time_ms-last_time_ms));
        telemetry.addLine();
        telemetry.addData("TargetPoint",targetPoint.toString());
        telemetry.addData("TargetClipHeadingRadian",targetClipHeadingRadian);
        telemetry.addData("TargetRadianAroundArm3",targetRadianAroundArm3);
        telemetry.addLine();
        telemetry.addData("Now Degree", Arrays.toString(sixServoArmState.getServoNowDegree()));
        telemetry.addData("target Degree", Arrays.toString(sixServoArmState.getServoTargetDegree()));
        telemetry.addData("Now Point",sixServoArmController.getCurrentPosition().toString());
        telemetry.addLine();
        telemetry.addLine("Press X to switch mode");
        telemetry.addData("Now Mode",TestMode?"Direct Servo Control":"Arm Control");
        if(TestMode){
            telemetry.addLine("Direct Servo Control Mode");
            telemetry.addData("Test Servo ID",test_servo_id);
            telemetry.addData("Test Servo Position",test_servo_positions[test_servo_id]);
            telemetry.addLine("Use A/B to change servo");
            telemetry.addLine("Use Left Stick Y to change position");
        }else{
            telemetry.addLine("Arm Control Mode");
            telemetry.addData("Target Point",targetPoint);
            telemetry.addLine("Use Left Stick to move target point");
            telemetry.addLine("Use Right Stick Y to move target point Z");
            telemetry.addLine("Dpad Left: Find Position");
            telemetry.addLine("Dpad Up: Go to Sample Position");
            telemetry.addLine("Dpad Right: Lock Clip");
            telemetry.addLine("Dpad Down: Go to Drop Position");
            telemetry.addLine("A: Drop Clip");
        }
        telemetry.update();
    }
    public void shoot(){
        if(gamepad2.a){
            targetSpeed = 0;
        }
        if(gamepad2.b){
            targetSpeed = initSpeed;
        }
        if(gamepad2.xWasPressed()){
            targetSpeed -= 50;
        }
        if(gamepad2.yWasPressed()){
            targetSpeed += 50;
        }
        shooter.shoot(targetSpeed);
    }
    public void setArm(){
        if(now_time_ms-last_set_time_ms>50){
            targetPoint.add(new Point3D(10*gamepad1.left_stick_x,-10*gamepad1.left_stick_y,-10*gamepad1.right_stick_y));
            targetPoint.clamp(300,Point3D.ZERO);

            targetClipHeadingRadian+=gamepad1.right_stick_x*0.1;
            targetClipHeadingRadian= Math.max(-Math.PI,Math.min(Math.PI,targetClipHeadingRadian));
            last_set_time_ms=now_time_ms;
        }
        if(gamepad1.dpadLeftWasPressed()){
            //find
            targetPoint = SearchPoint;
            targetClipHeadingRadian = -Math.PI / 2;
            targetRadianAroundArm3 = 0;
        }
        if(gamepad1.dpadUpWasPressed()){
            //go to sample
            ArmAction target = new ArmAction(0,0,0,150,0);//findCandidate.findCandidate();

            targetPoint = new Point3D(target.GoToX - deltaX, target.GoToY - deltaY, 10);
            targetClipHeadingRadian = -Math.PI / 2;
            targetRadianAroundArm3 = target.ClipAngle;
        }
        if(gamepad1.dpadRightWasPressed()){
            //lock
            sixServoArmController.setClip(true);

        }
        if(gamepad1.dpadDownWasPressed()){
            //go to drop position
            targetPoint = new Point3D(100, 100, 50);
            targetClipHeadingRadian = 0;
            targetRadianAroundArm3 = 0;
        }
        if(gamepad1.a){
            //drop
            sixServoArmController.setClip(false);
        }
        sixServoArmController.setTargetPosition(targetPoint,targetClipHeadingRadian,targetRadianAroundArm3);
    }
    public int test_servo_id=0;
    public double[] test_servo_positions={0,0,0,0,0,0};

    public static double[] servoZeroPositionDegree = {-28.92857,-63.529411,-43.71428,55.5882 , -77.83783783783785, 0};
    public static double[] servoRangeDegree = {321.42857, 264.70588, 257.142847142857142857, 264.70588, 243.24324324324328, 170};//舵机总旋转角度
    public static double[] Position1 ={0.09,0.24,0.87,0.47,0.32};
    public static double[] Degree1 ={0,0,180,180,0};
    public static double[] Position2 ={0.37,0.58,0.52,0.13,0.69};
    public static double[] Degree2 ={90,90,90,90,90};

    public void directServoControl(){
        if(gamepad1.aWasReleased()){
            test_servo_id++;
            if (test_servo_id>5)test_servo_id=0;
        }
        if(gamepad1.bWasReleased()){
            test_servo_id--;
            if (test_servo_id<0)test_servo_id=5;
        }
        if(now_time_ms-last_set_time_ms>50){
            test_servo_positions[test_servo_id]-=gamepad1.left_stick_y*0.05;
            test_servo_positions[test_servo_id]= Math.max(0,Math.min(1,test_servo_positions[test_servo_id]));
            last_set_time_ms=now_time_ms;
        }
        sixServoArmController.getOutputter().setPosition(test_servo_id,test_servo_positions[test_servo_id]);
        if(gamepad1.yWasReleased()){
            for (int i = 0; i <= 4; i++) {
                if(Position1[i]!=0&& Position2[i]!=0) {
                    servoRangeDegree[i] = (Degree1[i] - Degree2[i]) / (Position1[i] - Position2[i]);
                    servoZeroPositionDegree[i]= Degree1[i] - servoRangeDegree[i] * Position1[i];
                }
            }
        }
    }
    public long now_time_ms,last_time_ms,last_set_time_ms;
    Point3D targetPoint = new Point3D(0,100,20);
    double targetClipHeadingRadian=0;
    double targetRadianAroundArm3=0;
}
