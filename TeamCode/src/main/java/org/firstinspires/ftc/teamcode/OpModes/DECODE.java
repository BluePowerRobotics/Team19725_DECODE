package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.controllers.Sweeper;
import org.firstinspires.ftc.teamcode.controllers.Trigger;
import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.SolveShootPoint;

import java.io.BufferedReader;
import java.io.FileReader;


//泵赛季主程序
@TeleOp(name="DECODE", group="AAA_DECODE")
public class DECODE extends LinearOpMode {
    public enum TEAM_COLOR {
        RED,BLUE
    }
    TEAM_COLOR teamColor;
    public enum TRIGGER_STATUS {
        //for 大三角
        FULL_SPEED,
        //for 小三角
        LOWER_SPEED,
        CLOSE,
        EMERGENCY_STOP
    }
    TRIGGER_STATUS triggerStatus = TRIGGER_STATUS.CLOSE;
    public enum SWEEPER_STATUS {
        EAT,
        GIVE_ARTIFACT,
        OUTPUT,
        STOP
    }
    SWEEPER_STATUS sweeperStatus = SWEEPER_STATUS.STOP;
    public enum SHOOTER_STATUS {
        SHOOTING,
        EMERGENCY_STOP,
        STOP
    }
    SHOOTER_STATUS shooterStatus = SHOOTER_STATUS.STOP;
    public ChassisController chassis;
    public Sweeper sweeper;
    public ShooterAction shooter;
    public Trigger trigger;
    //
    public  int targetSpeed = 900;
    public Pose2d startPose = new Pose2d(0,0,0);
    void Init(){
        try (BufferedReader reader = new BufferedReader(new FileReader("/sdcard/FIRST/pose.txt"))) {
            String[] data = reader.readLine().split(",");
            startPose = new Pose2d(
                    Double.parseDouble(data[0]),
                    Double.parseDouble(data[1]),
                    Double.parseDouble(data[2])
            );
        } catch (Exception e) {
            startPose = new Pose2d(0, 0, 0); // 默认值
        }

        //todo set team color
        teamColor = TEAM_COLOR.BLUE;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sweeper = new Sweeper(hardwareMap);
        trigger = new Trigger(hardwareMap);
        shooter = new ShooterAction(hardwareMap, telemetry);
        chassis = new ChassisController(hardwareMap, new Point2D(startPose.position.x, startPose.position.y), startPose.heading.toDouble());
    }
    void Telemetry(){

        telemetry.addData("NoHeadModeStartError:",chassis.noHeadModeStartError);
        telemetry.addData("NoHeadMode",chassis.useNoHeadMode?"NoHead":"Manual");
        telemetry.addData("RunMode",chassis.runningToPoint?"RUNNING_TO_POINT":"MANUAL");
        telemetry.addData("shooterSTATUS", shooterStatus.toString());
        telemetry.addData("sweeperSTATUS", sweeperStatus.toString());
        telemetry.addData("Position(mm)",chassis.robotPosition.getData().getPosition(DistanceUnit.MM).toString());
        telemetry.addData("Position(inch)",chassis.robotPosition.getData().getPosition(DistanceUnit.INCH).toString());
        telemetry.addData("targetSpeed", targetSpeed);
        telemetry.addData("1-power * 1000", shooter.getPower() * 1000);
        telemetry.addData("1-speed", shooter.getCurrent_speed());
        telemetry.update();
    }
    void trigger(){
        switch (triggerStatus){
            case FULL_SPEED:
                trigger.full_speed();
                break;
            case LOWER_SPEED:
                trigger.lower_speed();
                break;
            case CLOSE:
                trigger.close();
                break;
            case EMERGENCY_STOP:
                trigger.emergencyStop();
                break;
        }
    }

    void shoot(){

        if(gamepad1.yWasPressed() || gamepad2.aWasPressed()){
            if(shooterStatus == SHOOTER_STATUS.SHOOTING){
                shooterStatus = SHOOTER_STATUS.EMERGENCY_STOP;
            }
            else{
                shooterStatus = SHOOTER_STATUS.SHOOTING;
            }
        }

        if(gamepad1.aWasPressed() || gamepad2.aWasPressed()){
            shooterStatus = SHOOTER_STATUS.STOP;
        }


        switch (shooterStatus){
            case SHOOTING:
                boolean ifHit =   false;//todo = chassisController.wheelSpeeds.length;
                if(ifHit){
                    shooterStatus = SHOOTER_STATUS.EMERGENCY_STOP;
                }
                else{
                    sweeperStatus = SWEEPER_STATUS.GIVE_ARTIFACT;
                    if(targetSpeed > 1000){
                        triggerStatus = TRIGGER_STATUS.LOWER_SPEED;
                    }
                    else{
                        triggerStatus = TRIGGER_STATUS.FULL_SPEED;
                    }
                    shooter.setShootSpeed(targetSpeed);
                }
                break;

            case EMERGENCY_STOP:
                triggerStatus = TRIGGER_STATUS.EMERGENCY_STOP;
                sweeperStatus = SWEEPER_STATUS.STOP;
                shooter.setShootSpeed(-400);
                break;

            case STOP:
                triggerStatus = TRIGGER_STATUS.CLOSE;
                shooter.setShootSpeed(0);
                break;
        }
    }
    void chassis(){

        Pose2d pose = chassis.robotPosition.mecanumDrive.localizer.getPose();

        double drive = -gamepad1.left_stick_y-gamepad1.right_stick_y; // 前后
        double strafe = gamepad1.left_stick_x; // 左右
        double rotate =-gamepad1.right_stick_x; // 旋转

        if(gamepad1.left_stick_button && gamepad1.right_stick_button){
            drive = 0;
            strafe = 0;
            rotate = 0;
            //校准
            //todo add 校准
        }


        if(gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()){
            if(teamColor == TEAM_COLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r1));
                targetSpeed = ShooterAction.speed2_2;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r1));
                targetSpeed = ShooterAction.speed2_2;
            }
        }
        if(gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()){
            if(teamColor == TEAM_COLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r2));
                targetSpeed = ShooterAction.speed25_25;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r2));
                targetSpeed = ShooterAction.speed25_25;
            }
        }
        if(gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()){
            if(teamColor == TEAM_COLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r3));
                targetSpeed = ShooterAction.speed3_3;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r3));
                targetSpeed = ShooterAction.speed3_3;
            }
        }
        if(gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()){
            if(teamColor == TEAM_COLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r4));
                targetSpeed = ShooterAction.speed25_55;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r4));
                targetSpeed = ShooterAction.speed25_55;
            }
        }
        if(gamepad2.xWasPressed()){
            if(teamColor == TEAM_COLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r5));
                targetSpeed = ShooterAction.speed35_55;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r5));
                targetSpeed = ShooterAction.speed35_55;
            }
        }


        chassis.gamepadInput(strafe, drive, rotate);
        if(gamepad1.xWasReleased()) chassis.exchangeNoHeadMode();
        //if(gamepad1.yWasReleased()) {
        //    chassis.setTargetPoint(new Pose2d(new Vector2d(0,0), Rotation2d.fromDouble(0)));
        //}
        telemetry.addData("y-power",drive);
        telemetry.addData("x-power",strafe);
        telemetry.addData("r-power",rotate);



        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    void sweeper(){
        if(gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()){
            if(sweeperStatus == SWEEPER_STATUS.EAT){
                sweeperStatus = SWEEPER_STATUS.STOP;
            }
            else{
                sweeperStatus = SWEEPER_STATUS.EAT;
            }

        }
        else if(gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()){
            if(sweeperStatus == SWEEPER_STATUS.OUTPUT){
                sweeperStatus = SWEEPER_STATUS.STOP;
            }
            else{
                sweeperStatus = SWEEPER_STATUS.OUTPUT;
            }
        }

        if(gamepad2.yWasPressed()){
            sweeperStatus = SWEEPER_STATUS.GIVE_ARTIFACT;
        }

        switch (sweeperStatus){
            case EAT:
                sweeper.Eat();
                break;
            case GIVE_ARTIFACT:
                sweeper.GiveArtifact();
                break;
            case OUTPUT:
                sweeper.output();
                break;
            case STOP:
                sweeper.stop();
                break;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        while (opModeIsActive()) {
            sweeper();
            trigger();
            shoot();
            chassis();
            Telemetry();
        }
    }
}
