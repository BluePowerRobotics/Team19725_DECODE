package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controllers.Sweeper;
import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.controllers.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@TeleOp(name="DECODE", group="OpModes")
public class DECODE extends LinearOpMode {
    public ChassisController chassisController;
    public Sweeper sweeper;
    public Shooter shooter1;
    public Shooter shooter2;
    public  int targetSpeed = 900;
    public static int speed2_2 = 900;
    public static int speed25_25 = 925;
    public static int speed3_3 = 975;
    public static int speed3_6 = 1400;
    public Pose2d startPose = new Pose2d(0,0,0);
    void Init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sweeper = new Sweeper(hardwareMap);
        shooter1 = new Shooter(hardwareMap, telemetry, "shooterMotor1", true);
        shooter2 = new Shooter(hardwareMap, telemetry, "shooterMotor2", false);
        chassisController = new ChassisController(hardwareMap, new Point2D(0,0), 0);
    }
    void Telemetry(){
        telemetry.addData("targetSpeed", targetSpeed);
        telemetry.addData("1-power * 1000", shooter1.getPower() * 1000);
        telemetry.addData("1-speed", shooter1.getCurrent_speed());
        telemetry.update();
    }
    void shoot(){

        if(gamepad1.y || gamepad2.a){
            boolean ifhit = false;// = chassisController.wheelSpeeds.length;
            if(ifhit){
                sweeper.stop();
                //banji -1
                shooter1.shoot(0);
                shooter2.shoot(0);
            }
            else{
                sweeper.GiveArtifact();
                //banji songqiu
                shooter1.shoot(targetSpeed);
                shooter2.shoot(targetSpeed);
            }

        }

        if(gamepad2.y){
            sweeper.GiveArtifact();
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        while (opModeIsActive()) {
            Telemetry();
        }
    }
}
