package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.controllers.turret.TurretAngleController;
import org.firstinspires.ftc.teamcode.controllers.turret.TurretController;
import org.firstinspires.ftc.teamcode.controllers.turret.model.TurretInfo;
import org.firstinspires.ftc.teamcode.utility.Point3D;

import java.util.List;

@TeleOp(name="TurretControllerTest", group="Test")
@Config
public class TurretControllerTest extends LinearOpMode {
    private TurretController turretController;
    private TurretAngleController turretAngleController;
    private ChassisController chassis;
    
    public static boolean useDRL = false;
    public static double targetX = 2.0;
    public static double targetY = 0.0;
    public static double targetZ = 0.5;
    public static double turretHeight = 0.1;
    public static double theta = Math.PI / 4;
    
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        turretController = new TurretController(telemetry);
        turretAngleController = new TurretAngleController(hardwareMap, telemetry, "turretServo", true);
        chassis = new ChassisController(hardwareMap);
        
        TurretController.TurretCalculator.Param.target = new Point3D(targetX, targetY, targetZ);
        TurretController.TurretCalculator.Param.turretHeight = turretHeight;
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("DRL Model", turretController.isDRLModelLoaded() ? "Loaded" : "Not Loaded");
        telemetry.addData("Target Position", String.format("(%.2f, %.2f, %.2f)", targetX, targetY, targetZ));
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        turretController.setUseDRLModel(useDRL);
        turretAngleController.setTo45Degrees();
        
        while (opModeIsActive() && !isStopRequested()) {
            handleInput();
            updateSystems();
            updateTelemetry();
        }
    }
    
    private void handleInput() {
        if (gamepad1.a) {
            useDRL = !useDRL;
            turretController.setUseDRLModel(useDRL);
            sleep(200);
        }
        
        if (gamepad1.b) {
            turretAngleController.setTo45Degrees();
            sleep(200);
        }
        
        if (gamepad1.x) {
            TurretController.TurretCalculator.Param.target = new Point3D(targetX, targetY, targetZ);
            TurretController.TurretCalculator.Param.turretHeight = turretHeight;
            telemetry.addData("Target Updated", String.format("(%.2f, %.2f, %.2f)", targetX, targetY, targetZ));
            telemetry.update();
            sleep(200);
        }
        
        if (gamepad1.y) {
            calculateAndDisplayTurretInfo();
            sleep(200);
        }
        
        if (gamepad1.dpad_up) {
            theta += Math.toRadians(5);
            theta = Math.min(Math.PI / 2, theta);
            sleep(200);
        }
        if (gamepad1.dpad_down) {
            theta -= Math.toRadians(5);
            theta = Math.max(0, theta);
            sleep(200);
        }
        
        chassis.gamepadInput(
            gamepad1.left_stick_x,
            gamepad1.left_stick_y,
            gamepad1.right_stick_x
        );
    }
    
    private void calculateAndDisplayTurretInfo() {
        Pose2d pose2d = chassis.robotPosition.getData().getPose2d();
        PoseVelocity2d poseVelocity2d = new PoseVelocity2d(
            new com.acmerobotics.roadrunner.Vector2d(0, 0),
            0
        );
        
        TurretController.TurretCalculator calculator = turretController.createCalculator(useDRL);
        
        List<TurretInfo> results = calculator.solveSpeedWithDRL(theta, pose2d, poseVelocity2d);
        
        telemetry.addData("Calculation Results", "Found " + results.size() + " solutions");
        for (int i = 0; i < results.size(); i++) {
            TurretInfo info = results.get(i);
            telemetry.addData("Solution " + (i + 1), info.toString());
        }
        telemetry.update();
    }
    
    private void updateSystems() {
        chassis.robotPosition.update();
        turretAngleController.update();
    }
    
    private void updateTelemetry() {
        telemetry.addData("DRL Mode", useDRL ? "Enabled" : "Disabled");
        telemetry.addData("Turret Angle", String.format("%.2f°", Math.toDegrees(theta)));
        telemetry.addData("Turret Current Angle", String.format("%.2f°", turretAngleController.getCurrentAngleDegrees()));
        telemetry.addData("Robot Position", String.format("(%.2f, %.2f)", 
            chassis.robotPosition.getData().getPose2d().position.x,
            chassis.robotPosition.getData().getPose2d().position.y));
        telemetry.addData("Target Position", String.format("(%.2f, %.2f, %.2f)", targetX, targetY, targetZ));
        telemetry.addData("Controls", 
            "A: Toggle DRL | B: Set 45° | X: Update Target | Y: Calculate");
        telemetry.addData("Angle Controls", 
            "DPad Up/Down: Adjust Theta");
        telemetry.update();
    }
}