package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.Shooter;
import org.firstinspires.ftc.teamcode.utility.SolveShootPoint;


@TeleOp
@Config
public class ShooterTest extends LinearOpMode {
    public SolveShootPoint solveShootPoint = new SolveShootPoint();
    public static double R = 48 * Math.sqrt(2);
    public static double x = 24;
    public static double y = 0;
    public Shooter shooter;
    public static double initSpeed = 300;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap, telemetry);
        telemetry.addData("RED:ANSX, ANSY, ANSTheta", solveShootPoint.solveREDShootPoint(new Pose2d(x, y, 0), R));
        telemetry.addData("BLUE:ANSX, ANSY, ANSTheta", solveShootPoint.solveBLUEShootPoint(new Pose2d(x, y, 0), R));

        telemetry.update();
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
                targetSpeed -= 50;
            }
            if(gamepad1.yWasPressed()){
                targetSpeed += 50;
            }
            shooter.shoot(targetSpeed);
            telemetry.update();
        }
    }
}
