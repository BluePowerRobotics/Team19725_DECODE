package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.Sweeper;
import org.firstinspires.ftc.teamcode.controllers.shooter.Shooter;


@TeleOp(name = "shooterTest", group = "TEST")
@Config
public class ShooterTest extends LinearOpMode {
    //2-2   900
    // 2.5-2.5 925
    public static int TimePerFrame = 20;
    public Sweeper sweeper;
    public Shooter shooter1;
    public Shooter shooter2;
    public static double initSpeed = 2500;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sweeper = new Sweeper(hardwareMap);
        shooter1 = new Shooter(hardwareMap, telemetry, "shooterMotor1", true);
        shooter2 = new Shooter(hardwareMap, telemetry, "shooterMotor2", false);
//        telemetry.addData("RED:ANSX, ANSY, ANSTheta", solveShootPoint.solveREDShootPoint(new Pose2d(x, y, 0), R));
//        telemetry.addData("BLUE:ANSX, ANSY, ANSTheta", solveShootPoint.solveBLUEShootPoint(new Pose2d(x, y, 0), R));
//
//        telemetry.update();
        waitForStart();
        double targetSpeed = 0;
        while (opModeIsActive()) {
            if(gamepad1.a){
                targetSpeed = initSpeed;
            }
            if(gamepad1.b){
                targetSpeed = 0;
            }
            if(gamepad1.xWasPressed()){
                targetSpeed -= 100;
            }
            if(gamepad1.yWasPressed()){
                targetSpeed += 100;
            }
            shooter1.shoot(targetSpeed);
            shooter2.shoot(targetSpeed);
            if(gamepad1.dpadLeftWasPressed()){
                sweeper.GiveArtifact();
            }
            telemetry.addData("targetSpeed", targetSpeed);
            telemetry.addData("1-postion", shooter1.getCurrent_encoder());
            telemetry.addData("1-power", shooter1.getPower());
            telemetry.addData("1-speed", shooter1.getCurrent_speed());
            telemetry.addData("2-postion", shooter2.getCurrent_encoder());
            telemetry.addData("2-power", shooter2.getPower());
            telemetry.addData("2-speed", shooter2.getCurrent_speed());
            telemetry.update();
            sleep(TimePerFrame);
        }
    }
}
