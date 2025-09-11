package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;
import org.firstinspires.ftc.teamcode.controllers.Point3D;

import org.firstinspires.ftc.teamcode.Vision.FindCandidate;

@TeleOp(name = "SixServoArmCalculationTester", group = "Test")
public class SixServoArmCalculationTester extends LinearOpMode {
    FindCandidate findCandidate = new FindCandidate();
    SixServoArmController sixServoArmController;
    SixServoArmOutputter sixServoArmOutputter;
    SixServoArmState sixServoArmState;
    public void initiate(){
        findCandidate.init(hardwareMap, telemetry, 0);
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
            setArm();
        }
    }
    public void addTele(){
        last_time_ms=now_time_ms;
        now_time_ms=System.currentTimeMillis();
        telemetry.addData("FPS",1000.0/(now_time_ms-last_time_ms));
        telemetry.addLine();
        telemetry.addData("TargetPoint",targetPoint);
        telemetry.addData("TargetClipHeadingRadian",targetClipHeadingRadian);
        telemetry.addData("TargetRadianAroundArm3",targetRadianAroundArm3);
        telemetry.addLine();
        telemetry.addData("Now Degree",sixServoArmState.getServoNowDegree());
        telemetry.addData("target Degree",sixServoArmState.getServoTargetDegree());
        telemetry.addData("Now Point",sixServoArmController.getCurrentPosition());
        telemetry.update();
    }
    public void setArm(){
        if(now_time_ms-last_set_time_ms>50){
            targetPoint.add(new Point3D(10*gamepad2.left_stick_x,-10*gamepad2.left_stick_y,-10*gamepad2.right_stick_y));
            targetPoint.clamp(300,Point3D.ZERO);
            last_set_time_ms=now_time_ms;
        }
        if(gamepad1.dpadLeftWasPressed()){
            //find
            targetPoint = new Point3D(0, 150, 100);
            targetClipHeadingRadian = -Math.PI / 2;
            targetRadianAroundArm3 = 0;
        }
        if(gamepad1.dpadUpWasPressed()){
            //go to sample
            ArmAction target = findCandidate.findCandidate();

            //???
            targetPoint = new Point3D(target.GoToX, target.GoToY, 10);
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

    public long now_time_ms,last_time_ms,last_set_time_ms;
    Point3D targetPoint = new Point3D(0,100,20);
    double targetClipHeadingRadian=0;
    double targetRadianAroundArm3=0;
}
