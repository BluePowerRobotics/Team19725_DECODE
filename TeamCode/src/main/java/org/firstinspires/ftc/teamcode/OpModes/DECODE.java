package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.KalmanFusionLocalizer;
import org.firstinspires.ftc.teamcode.controllers.BlinkinLedController;
import org.firstinspires.ftc.teamcode.controllers.DisSensor;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.LedPreset;
import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper_PID;
import org.firstinspires.ftc.teamcode.controllers.Trigger;
import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.SolveShootPoint;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetector;

import java.io.BufferedReader;
import java.io.FileReader;


//泵赛季主程序
@Config
@TeleOp(name="DECODE", group="AAA_DECODE")

public class DECODE extends LinearOpMode {
    long lastNanoTime;
    public enum ROBOT_STATUS{
        EATING,
        WAITING,
        OUTPUTTING,
        SHOOTING,
        EMERGENCY_STOP,
        CLIMBING
    }

    ROBOT_STATUS robotStatus = ROBOT_STATUS.WAITING;
    public enum TEAM_COLOR {
        RED,BLUE
    }
    TEAM_COLOR teamColor;
    public enum TRIGGER_STATUS {
        OPEN,
        CLOSE
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
        STOP
    }
    SHOOTER_STATUS shooterStatus = SHOOTER_STATUS.STOP;
    public ChassisController chassis;
    public Sweeper_PID sweeper;
    public ShooterAction shooter;
    public Trigger trigger;
    public ActionRunner actionRunner;
    public DisSensor disSensor;
    public BlinkinLedController ledController;
    AprilTagDetector aprilTagDetector;
    //
    public  int targetSpeed = ShooterAction.speed2_2;
    public Pose2d startPose = new Pose2d(0,0,0);
    public static int additionSpeed = 25;
    int n = 0;
    boolean ifBacking = false;
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
        telemetry = InstanceTelemetry.init(telemetry);
        sweeper = new Sweeper_PID(hardwareMap, telemetry, "sweeperMotor", true);
        trigger = new Trigger(hardwareMap);
        shooter = new ShooterAction(hardwareMap, telemetry);
        chassis = new ChassisController(hardwareMap, startPose);
        disSensor = new DisSensor(hardwareMap);
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap);
        actionRunner = new ActionRunner();
        ledController = new BlinkinLedController(hardwareMap);
    }
    void inputRobotStatus(){
        if(gamepad2.startWasPressed()){
            actionRunner = new ActionRunner();
            actionRunner.add(sweeper.SweeperBack());
        }
        if(gamepad1.yWasPressed() || gamepad2.yWasPressed()){
            if(robotStatus == ROBOT_STATUS.SHOOTING){
                robotStatus = ROBOT_STATUS.EMERGENCY_STOP;
            }
            else{
                robotStatus = ROBOT_STATUS.SHOOTING;
            }
        }

        if(gamepad1.aWasPressed() || gamepad2.aWasPressed()){
            robotStatus = ROBOT_STATUS.WAITING;
        }

        if(gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()){
            if(robotStatus == ROBOT_STATUS.EATING){
                robotStatus = ROBOT_STATUS.WAITING;
            }
            else{
                robotStatus = ROBOT_STATUS.EATING;
            }

        }
        else if(gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()){
            if(robotStatus == ROBOT_STATUS.OUTPUTTING){
                robotStatus = ROBOT_STATUS.WAITING;
            }
            else{
                robotStatus = ROBOT_STATUS.OUTPUTTING;
            }
        }
        if(gamepad2.bWasPressed()){
            n += 1;
            gamepad2.rumble(0,300,150);
        }
        if(gamepad2.xWasPressed()){
            n -= 1;
            gamepad2.rumble(300,0,150);
        }
    }
    void setStatus(){
        switch (robotStatus) {
            case EATING:
                //如果吸满了，自动切换到waiting状态
                if(disSensor.Whether_full()){
                    robotStatus = ROBOT_STATUS.WAITING;
                }
                sweeperStatus = SWEEPER_STATUS.EAT;
                shooterStatus = SHOOTER_STATUS.STOP;
                triggerStatus = TRIGGER_STATUS.CLOSE;
                ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case WAITING:
                boolean AprilTagStatus = false;
                if(!Double.isNaN(aprilTagDetector.getPose().pose.position.x)){
                    AprilTagStatus = true;
                }
                sweeperStatus = SWEEPER_STATUS.STOP;
                shooterStatus = SHOOTER_STATUS.STOP;
                triggerStatus = TRIGGER_STATUS.CLOSE;
                if(teamColor == TEAM_COLOR.RED){
                    if(AprilTagStatus){
                        ledController.setColor(LedPreset.HEARTBEAT_RED.getPattern());
                    }
                    else{
                        ledController.showRedTeam();
                    }
                }
                else{
                    if(AprilTagStatus){
                        ledController.setColor(LedPreset.HEARTBEAT_BLUE.getPattern());
                    }
                    else{
                        ledController.showBlueTeam();
                    }
                }
                break;
            case SHOOTING:
                ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                shooterStatus = SHOOTER_STATUS.SHOOTING;
                //sweeper和trigger状态由shooter条件决定，在shoot()中
                break;
            case EMERGENCY_STOP:
                ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                shooterStatus = SHOOTER_STATUS.STOP;
                sweeperStatus = SWEEPER_STATUS.STOP;
                triggerStatus = TRIGGER_STATUS.CLOSE;
                break;
            case OUTPUTTING:
                ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                sweeperStatus = SWEEPER_STATUS.OUTPUT;
                shooterStatus = SHOOTER_STATUS.STOP;
                triggerStatus = TRIGGER_STATUS.CLOSE;
                break;

        }
        double realTargetSpeed = targetSpeed + n * additionSpeed;
        if(!actionRunner.isBusy()){
            if(chassis.getUseNoHeadMode()){
                //useNoHeadMode
                //LED常亮
                if(realTargetSpeed<=700)ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                else if(realTargetSpeed<=725)ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                else if(realTargetSpeed<=750)ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                else if(realTargetSpeed<=800)ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                else ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }else{
                long currentTimeMS=System.currentTimeMillis();
                //roboticBasedMode
                //LED闪烁
                if(currentTimeMS-lastSetTimeMS>500/*切换间隔，毫秒*/){
                    showSpeedColor=!showSpeedColor;
                    lastSetTimeMS=currentTimeMS;
                }
                if(!showSpeedColor) {
                    if (teamColor == TEAM_COLOR.BLUE) {
                        ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                    }
                    else if (teamColor == TEAM_COLOR.RED) {
                        ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
                    }
                }else{
                    if(realTargetSpeed<=650)ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    else if(realTargetSpeed<=725)ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                    else if(realTargetSpeed<=800)ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    else if(realTargetSpeed<=875)ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    else ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                }
            }
        }


        else{
            ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }

    }
    long lastSetTimeMS=0;
    boolean showSpeedColor=false;
    void Telemetry(){
        telemetry.addData("targetSpeed", targetSpeed + n * additionSpeed);
        telemetry.addData("NoHeadMode",chassis.getUseNoHeadMode()?"PlayerBased":"RoboticBased");
        telemetry.addData("isBusy", actionRunner.isBusy());
        telemetry.addData("TeamColor", teamColor);
        telemetry.addData("RobotSTATUS", robotStatus.toString());
        telemetry.addData("n", n);
        telemetry.addData("NoHeadModeStartError:",chassis.noHeadModeStartError);
        telemetry.addData("RunMode",chassis.runningToPoint?"RUNNING_TO_POINT":"MANUAL");
//        telemetry.addData("shooterSTATUS", shooterStatus.toString());
//        telemetry.addData("sweeperSTATUS", sweeperStatus.toString());
//        telemetry.addData("triggerSTATUS", triggerStatus.toString());
//        telemetry.addData("Position(mm)",chassis.robotPosition.getData().getPosition(DistanceUnit.MM).toString());
        telemetry.addData("SweeperSpeeed", sweeper.getCurrent_speed());
        telemetry.addData("SweeperPower * 1000", sweeper.getPower() * 1000);
        telemetry.addData("Heading", chassis.robotPosition.getData().headingRadian);
        telemetry.addData("targetHeading",chassis.getHeadingLockRadian());
        telemetry.addData("Position(inch)", Point2D.rotate(chassis.robotPosition.getData().getPosition(DistanceUnit.INCH),teamColor==TEAM_COLOR.BLUE?Math.PI/2:-Math.PI/2).toString());
        telemetry.addData("1-power * 1000", shooter.getPower1() * 1000);
        telemetry.addData("2-power * 1000", shooter.getPower2() * 1000);
        telemetry.addData("1-speed", shooter.getCurrent_speed1());
        telemetry.addData("2-speed", shooter.getCurrent_speed2());
        telemetry.addData("FPS",1000000000.0/(System.nanoTime()-lastNanoTime));
        telemetry.update();
        lastNanoTime=System.nanoTime();
    }
    void trigger(){
        switch (triggerStatus){
            case OPEN:
                trigger.open();
                break;
            case CLOSE:
                trigger.close();
                break;
        }
    }

    void shoot(){
        switch (shooterStatus){
            case SHOOTING:
                boolean ifHit =   false;//todo = chassisController.wheelSpeeds.length;
                if(ifHit){
                    robotStatus = ROBOT_STATUS.EMERGENCY_STOP;
                }
                else{
                    boolean hasReachedTargetSpeed = shooter.setShootSpeed(targetSpeed + n * additionSpeed);
                    if(hasReachedTargetSpeed){
                        sweeperStatus = SWEEPER_STATUS.GIVE_ARTIFACT;
                        triggerStatus = TRIGGER_STATUS.OPEN;
                    }
                }
                break;

            case STOP:
                shooter.setShootSpeed(0);
                break;
        }
    }
    public static double time=1.2;
    void chassis(){

        Pose2d pose = chassis.robotPosition.getData().getPose2d();

        double drive = -time*gamepad1.left_stick_y - 0.3 * gamepad2.left_stick_y; // 前后
        double strafe = time*gamepad1.left_stick_x + 0.3 * gamepad2.left_stick_x; // 左右
        double rotate =-time*gamepad1.right_stick_x  - 0.3 * gamepad2.right_stick_x; // 旋转

        if(gamepad1.left_stick_button && gamepad1.right_stick_button){
            drive = 0;
            strafe = 0;
            rotate = 0;
            //校准
            //todo add 校准

            chassis.resetPosition(aprilTagDetector.getPose().pose);
        }

        //自动对准
        if(gamepad1.left_trigger > 0.6 || gamepad2.left_trigger > 0.6){
            double heading = 0;
            if(teamColor == TEAM_COLOR.RED){
                heading = SolveShootPoint.solveREDShootHeading(pose);
                //我认为没啥必要 先去睡觉了 电池充好了记得绑黑色扎带
                targetSpeed = ShooterAction.speed25_55;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                heading = SolveShootPoint.solveBLUEShootHeading(pose);
                targetSpeed = ShooterAction.speed25_55;
            }
            chassis.setHeadingLockRadian(heading);
        }

        //自动对准人玩区
        if(gamepad1.right_trigger > 0.6 || gamepad2.right_trigger > 0.6){
            double heading = 0;
            if(teamColor == TEAM_COLOR.RED){
                heading = -Math.PI / 2;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                heading = Math.PI / 2;
            }
            chassis.setHeadingLockRadian(heading);
        }

        //自瞄
        if(gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()){
            n = 0;
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
            n = 0;
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
            n = 0;
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
            n = 0;
            if(teamColor == TEAM_COLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r4));
                targetSpeed = ShooterAction.speed25_55;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r4));
                targetSpeed = ShooterAction.speed25_55;
            }
        }
//        if(gamepad2.xWasPressed()){
//            if(teamColor == TEAM_COLOR.RED){
//                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r5));
//                targetSpeed = ShooterAction.speed35_55;
//            }
//            if(teamColor == TEAM_COLOR.BLUE){
//                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r5));
//                targetSpeed = ShooterAction.speed35_55;
//            }
//        }


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


//        if(gamepad2.yWasPressed()){
//            sweeperStatus = SWEEPER_STATUS.GIVE_ARTIFACT;
//        }
        switch (sweeperStatus){
            case EAT:
                sweeper.Sweep(Sweeper_PID.EatVel);
                break;
            case GIVE_ARTIFACT:
                sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
                break;
            case OUTPUT:
                sweeper.output();
                break;
            case STOP:
                if(!actionRunner.isBusy()){
                    sweeper.Sweep(0);
                }
                break;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        while(opModeInInit()){
            if(gamepad1.a){
                teamColor = TEAM_COLOR.BLUE;
            }
            if(gamepad1.b){
                teamColor = TEAM_COLOR.RED;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                ledController.showBlueTeam();
            }
            if(teamColor == TEAM_COLOR.RED){
                ledController.showRedTeam();
            }
            if(teamColor == TEAM_COLOR.BLUE){
                chassis.resetNoHeadModeStartError(-Math.PI/2);
            }
            else{
                chassis.resetNoHeadModeStartError(Math.PI/2);
            }
            telemetry.addData("TEAM_COLOR", teamColor.toString());
            telemetry.update();
        }
        waitForStart();
        lastNanoTime=System.nanoTime();
        lastSetTimeMS=System.currentTimeMillis();
        while (opModeIsActive()) {
            inputRobotStatus();
            setStatus();
            shoot();
            sweeper();
            trigger();
            Telemetry();
            chassis();

            if(actionRunner.isBusy()){
                actionRunner.update();
            }
        }
    }
}
