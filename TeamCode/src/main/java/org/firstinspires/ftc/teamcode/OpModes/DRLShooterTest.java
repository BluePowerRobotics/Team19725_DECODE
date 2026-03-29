package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper_PID;
import org.firstinspires.ftc.teamcode.controllers.Trigger;
import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;
import org.firstinspires.ftc.teamcode.controllers.turret.TurretAngleController;

@TeleOp(name="DRLShooterTest", group="Test")
@Config
public class DRLShooterTest extends LinearOpMode {
    private ShooterAction shooter;
    private TurretAngleController turretController;
    private ChassisController chassis;
    private Sweeper_PID sweeper;
    private Trigger trigger;
    
    public static boolean useDRL = false;
    public static double targetX = 72;
    public static double targetY = 0;
    public static boolean useServo = true;
    
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        shooter = new ShooterAction(hardwareMap, telemetry);
        turretController = new TurretAngleController(hardwareMap, telemetry, "turretServo", useServo);
        chassis = new ChassisController(hardwareMap);
        sweeper = new Sweeper_PID(hardwareMap, telemetry, "sweeperMotor", false);
        trigger = new Trigger(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("DRL Model", shooter.isDRLModelLoaded() ? "Loaded" : "Not Loaded");
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        turretController.setTo45Degrees();
        shooter.setUseDRL(useDRL);
        
        while (opModeIsActive() && !isStopRequested()) {
            handleInput();
            updateSystems();
            updateTelemetry();
        }
    }
    
    private void handleInput() {
        if (gamepad1.a) {
            useDRL = !useDRL;
            shooter.setUseDRL(useDRL);
            sleep(200);
        }
        
        if (gamepad1.b) {
            turretController.setTo45Degrees();
            sleep(200);
        }
        
        if (gamepad1.x) {
            // 获取机器人位置
            double robotX = chassis.robotPosition.getData().getPose2d().position.x;
            double robotY = chassis.robotPosition.getData().getPose2d().position.y;
            
            // 计算目标相对于机器人的位置
            double targetRelX = targetX - robotX;
            double targetRelY = targetY - robotY;
            
            // 获取机器人速度（假设从ChassisController获取）
            double robotVx = 0; // 这里需要根据实际情况获取机器人速度
            double robotVy = 0; // 这里需要根据实际情况获取机器人速度
            
            // 更新ShooterAction中的参数
            shooter.robotVx = robotVx;
            shooter.robotVy = robotVy;
            shooter.targetRelX = targetRelX;
            shooter.targetRelY = targetRelY;
            
            telemetry.addData("Position Updated", String.format("Robot: (%.2f, %.2f), Target Rel: (%.2f, %.2f)", robotX, robotY, targetRelX, targetRelY));
            telemetry.update();
            sleep(200);
        }
        
        if (gamepad1.y) {
            int targetSpeed = ShooterAction.targetSpeed_high;
            if (useDRL) {
                double[] launchVelocity = shooter.shooter_Left.calculateDRLSpeed(
                    shooter.robotVx, shooter.robotVy, 
                    shooter.targetRelX, shooter.targetRelY
                );
                int drlSpeed = shooter.shooter_Left.calculateLaunchSpeedFromVector(launchVelocity);
                if (drlSpeed > 0) {
                    targetSpeed = drlSpeed;
                }
            }
            
            shooter.setShootSpeed(targetSpeed);
            trigger.open();
            sleep(100);
            sweeper.SweeperAction(Sweeper_PID.GiveTheArtifactVel);
            sleep(1000);
            sweeper.Sweep(0);
            trigger.close();
            shooter.setShootSpeed(0);
        }
        
        if (gamepad1.dpad_up) {
            turretController.setAngleDegrees(60);
        }
        if (gamepad1.dpad_down) {
            turretController.setAngleDegrees(30);
        }
        if (gamepad1.dpad_left) {
            turretController.setAngleDegrees(45);
        }
        if (gamepad1.dpad_right) {
            turretController.setAngleDegrees(45);
        }
        
        if (gamepad1.left_bumper) {
            sweeper.SweeperAction(Sweeper_PID.EatVel);
        } else if (gamepad1.right_bumper) {
            sweeper.SweeperAction(Sweeper_PID.OutputVel);
        } else {
            sweeper.Sweep(0);
        }
        
        chassis.gamepadInput(
            gamepad1.left_stick_x,
            gamepad1.left_stick_y,
            gamepad1.right_stick_x
        );
    }
    
    private void updateSystems() {
        chassis.robotPosition.update();
        turretController.update();
    }
    
    private void updateTelemetry() {
        telemetry.addData("DRL Mode", useDRL ? "Enabled" : "Disabled");
        telemetry.addData("Turret Angle", String.format("%.2f°", turretController.getCurrentAngleDegrees()));
        telemetry.addData("Robot Position", String.format("(%.2f, %.2f)", 
            chassis.robotPosition.getData().getPose2d().position.x,
            chassis.robotPosition.getData().getPose2d().position.y));
        telemetry.addData("Robot Velocity", String.format("(%.2f, %.2f)", 
            shooter.robotVx, shooter.robotVy));
        telemetry.addData("Target Position", String.format("(%.2f, %.2f)", targetX, targetY));
        telemetry.addData("Target Rel Position", String.format("(%.2f, %.2f)", 
            shooter.targetRelX, shooter.targetRelY));
        telemetry.addData("Shooter Speed 1", String.format("%.0f", shooter.getCurrent_speed1()));
        telemetry.addData("Shooter Speed 2", String.format("%.0f", shooter.getCurrent_speed2()));
        telemetry.addData("Controls", 
            "A: Toggle DRL | B: Set 45° | X: Update Pos | Y: Shoot");
        telemetry.addData("Angle Controls", 
            "DPad: 30°/45°/60°");
        telemetry.update();
    }
}