package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.controllers.Point3D;

@TeleOp(name = "SixServoArmCalculationTester", group = "Test")
public class SixServoArmCalculationTester extends LinearOpMode {
    SixServoArmController sixServoArmController;
    SixServoArmOutputter sixServoArmOutputter;
    SixServoArmState sixServoArmState;
    public void initiate(){
        SixServoArmController.setInstance(hardwareMap,telemetry);
        sixServoArmController=SixServoArmController.getInstance();
        sixServoArmOutputter = sixServoArmController.getOutputter();
        sixServoArmState = sixServoArmController.getState();
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
        telemetry.addData("ServoNowDegree",sixServoArmState.getServoNowDegree());
        telemetry.update();
    }
    public void setArm(){
        if(now_time_ms-last_set_time_ms>50){
            targetPoint.add(new Point3D(10*gamepad2.left_stick_x,-10*gamepad2.left_stick_y,-10*gamepad2.right_stick_y));
            targetPoint.clamp(300,Point3D.ZERO);
            last_set_time_ms=now_time_ms;
        }
        sixServoArmController.setTargetPosition(targetPoint,targetClipHeadingRadian,targetRadianAroundArm3);
    }

    public long now_time_ms,last_time_ms,last_set_time_ms;
    Point3D targetPoint = new Point3D(0,100,20);
    double targetClipHeadingRadian=0;
    double targetRadianAroundArm3=0;
}
