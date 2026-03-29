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
import org.firstinspires.ftc.teamcode.controllers.BlinkinLedController;
import org.firstinspires.ftc.teamcode.controllers.DisSensor;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.LedPreset;
import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper_PID;
import org.firstinspires.ftc.teamcode.controllers.Trigger;
import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.controllers.elevator.ElevatorController;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.solvepoint.SolveClimbPoint;
import org.firstinspires.ftc.teamcode.utility.solvepoint.SolveEatPoint;
import org.firstinspires.ftc.teamcode.utility.solvepoint.SolveShootPoint;
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
    
    // DRL 模式枚举
    public enum DRL_MODE {
        FullDRL,  // 使用 DRL 计算速度
        RunDRL,   // 小车速度大于预设值时使用 DRL，否则使用传统方法
        NoDRL     // 使用传统方法
    }
    DRL_MODE drlMode = DRL_MODE.RunDRL;  // 默认使用 RunDRL 模式
    public static double drlSpeedThreshold = 0.1;  // 速度阈值（m/s）
    public ChassisController chassis; // 底盘控制器实例，负责机器人的移动控制
    public Sweeper_PID sweeper; // 清扫器控制器实例
    public ShooterAction shooter; // 发射器控制器实例
    public Trigger trigger; // 触发器控制器实例
    public ElevatorController elevatorController; // 电梯控制器实例
    public ActionRunner actionRunner; // 动作运行器实例
    public BlinkinLedController ledController; // LED控制器实例
//    AprilTagDetector aprilTagDetector;
    //暂时关闭发射时的速度限制
    public static int OpenSweeperSpeedThreshold = 1000;
    //
    public double targetSpeed = ShooterAction.speed35_55;
    public Pose2d startPose = new Pose2d(0,0,0);
    public static double AdditionK = 0.01;
    //targetSpeed乘上Kspeed才是真实速度，修正发射速度
    double Kspeed = 1;
    boolean directControl=false;
    int currentPosition = 0;
    //修正角度偏差（从读取的底层修改）
    //todo 让degreeOffset生效 @gyw
    public double degreeOffset = 0;
    public static double AdditionDegree = 1.0;
    public static double startShootingHeading = Math.PI / 2;
    public static double toleranceHeading = 0.068;
    public static double KtoleranceHeading = 1;
    public boolean ReadyToShoot = false;
    public  Pose2d BlueResetPose = new Pose2d(72 - ChassisController.PARAMS.SideToCenterInch,72 - ChassisController.PARAMS.FrontToCenterInch, Math.PI / 2);
    public  Pose2d RedResetPose = new Pose2d(72 - ChassisController.PARAMS.SideToCenterInch,-72 + ChassisController.PARAMS.FrontToCenterInch, -Math.PI / 2);

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
        teamColor = TEAM_COLOR.RED;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);
        sweeper = new Sweeper_PID(hardwareMap, telemetry, "sweeperMotor", false);
        trigger = new Trigger(hardwareMap);
        shooter = new ShooterAction(hardwareMap, telemetry);
        chassis = new ChassisController(hardwareMap, startPose);
        elevatorController = new ElevatorController(hardwareMap);
//        aprilTagDetector = new AprilTagDetector();
//        aprilTagDetector.init(hardwareMap);
        actionRunner = new ActionRunner();
        ledController = new BlinkinLedController(hardwareMap);
        
        // 初始化 DRL
        SolveShootPoint.initDRL(telemetry);
        SolveShootPoint.setUseDRLModel(true); // 初始启用 DRL
        ShooterAction.useDRL = true; // 初始启用 DRL
    }
    // 计算发射速度，根据当前 DRL 模式
    private int calculateShootSpeed(double distance) {
        switch (drlMode) {
            case FullDRL:
                // 始终使用 DRL
                return calculateShootSpeedWithDRL(distance);
            case RunDRL:
                // 检查机器人速度是否超过阈值
                double robotSpeed = Math.sqrt(
                    Math.pow(chassis.getVelocityX(), 2) +
                    Math.pow(chassis.getVelocityY(), 2)
                );
                if (robotSpeed > drlSpeedThreshold) {
                    // 速度超过阈值，使用 DRL
                    return calculateShootSpeedWithDRL(distance);
                } else {
                    // 速度低于阈值，使用传统方法
                    return SolveShootPoint.solveShootSpeed(distance);
                }
            case NoDRL:
            default:
                // 始终使用传统方法
                return SolveShootPoint.solveShootSpeed(distance);
        }
    }
    
    // 使用 DRL 计算发射速度
    private int calculateShootSpeedWithDRL(double distance) {
        // 计算目标相对位置
        double targetX = 0; // 目标绝对 x 坐标
        double targetY = 0; // 目标绝对 y 坐标
        
        // 根据队伍颜色设置目标坐标
        if (teamColor == TEAM_COLOR.RED) {
            // 红色队伍的目标位置
            targetX = -72;
            targetY = 72;
        } else if (teamColor == TEAM_COLOR.BLUE) {
            // 蓝色队伍的目标位置
            targetX = -72;
            targetY = -72;
        }
        
        double robotX = pose.position.x;
        double robotY = pose.position.y;
        double targetRelX = targetX - robotX;
        double targetRelY = targetY - robotY;
        
        // 计算机器人速度
        double robotVx = chassis.getVelocityX();
        double robotVy = chassis.getVelocityY();
        
        // 使用 DRL 计算速度
        return SolveShootPoint.solveShootSpeed(
            distance, robotVx, robotVy, targetRelX, targetRelY
        );
    }
    
    void inputRobotStatus(){
        if(gamepad1.dpadRightWasPressed()){
            KtoleranceHeading += 0.5;
        }
        if(gamepad1.dpadLeftWasPressed()){
            if (teamColor == TEAM_COLOR.RED) {
                targetSpeed = calculateShootSpeed(SolveShootPoint.solveREDShootDistance(pose));
            }
            if (teamColor == TEAM_COLOR.BLUE) {
                targetSpeed = calculateShootSpeed(SolveShootPoint.solveBLUEShootDistance(pose));
            }
        }

//        if(gamepad1.dpadUpWasPressed()){
//            if(teamColor == TEAM_COLOR.BLUE){
//                chassis.resetPosition(BlueResetPose);
//            }
//            if(teamColor == TEAM_COLOR.RED){
//                chassis.resetPosition(RedResetPose);
//            }
//        }
        if(gamepad2.xWasPressed()){
            robotStatus = ROBOT_STATUS.WAITING;
            actionRunner = new ActionRunner();
            actionRunner.add(sweeper.SweeperBack());
        }



        //二操的修正功能
        if(gamepad2.dpadLeftWasPressed()){
            degreeOffset -= AdditionDegree;
            chassis.robotPosition.mecanumDrive.localizer.setPose(new Pose2d(chassis.robotPosition.getData().getPose2d().position, chassis.robotPosition.getData().headingRadian + Math.toRadians(-AdditionDegree)));
        }
        if(gamepad2.dpadRightWasPressed()){
            degreeOffset += AdditionDegree;
            chassis.robotPosition.mecanumDrive.localizer.setPose(new Pose2d(chassis.robotPosition.getData().getPose2d().position, chassis.robotPosition.getData().headingRadian + Math.toRadians(AdditionDegree)));
        }
        if(gamepad2.dpadDownWasPressed()){
            Kspeed -= AdditionK;
        }
        if(gamepad2.dpadUpWasPressed()){
            Kspeed += AdditionK;
        }
        if(gamepad2.yWasPressed() && robotStatus != ROBOT_STATUS.CLIMBING){
            robotStatus = ROBOT_STATUS.SHOOTING;
        }

        if((gamepad1.aWasPressed() || gamepad2.aWasPressed())  && robotStatus != ROBOT_STATUS.CLIMBING){
            ReadyToShoot = false;
            robotStatus = ROBOT_STATUS.WAITING;
        }

        //一操 二操切换 二操可强制开启
        if(gamepad2.leftBumperWasPressed() && robotStatus != ROBOT_STATUS.CLIMBING){
            ReadyToShoot = false;
            if(robotStatus == ROBOT_STATUS.EATING){
                robotStatus = ROBOT_STATUS.WAITING;
            }
            else{
                robotStatus = ROBOT_STATUS.EATING;
            }

        }

        if(gamepad1.leftBumperWasPressed() && robotStatus != ROBOT_STATUS.CLIMBING){
            ReadyToShoot = false;
            robotStatus = ROBOT_STATUS.EATING;
        }
        else if((gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) && robotStatus != ROBOT_STATUS.CLIMBING){
            ReadyToShoot = false;
            if(robotStatus == ROBOT_STATUS.OUTPUTTING){
                robotStatus = ROBOT_STATUS.WAITING;
            }
            else{
                robotStatus = ROBOT_STATUS.OUTPUTTING;
            }
        }

        if(gamepad1.bWasPressed()){
            ReadyToShoot = false;
            if(robotStatus != ROBOT_STATUS.CLIMBING){
                robotStatus = ROBOT_STATUS.CLIMBING;
            }
            else{
                robotStatus = ROBOT_STATUS.WAITING;
            }
        }

        if(gamepad1.right_trigger > 0.6){
            double heading = 0;
            if (teamColor == TEAM_COLOR.RED) {
                heading = SolveEatPoint.solveREDEatHeading(pose);
            }
            if (teamColor == TEAM_COLOR.BLUE) {
                heading = SolveEatPoint.solveBLUEEatHeading(pose);
            }
            //todo
            chassis.setHeadingLockRadian(heading);
        }

        if(gamepad1.yWasPressed()){
            ReadyToShoot = !ReadyToShoot;
            if(ReadyToShoot){
                double heading = 0;
                if (teamColor == TEAM_COLOR.RED) {
                    heading = SolveShootPoint.solveREDShootHeading(pose);
                    targetSpeed = calculateShootSpeed(SolveShootPoint.solveREDShootDistance(pose));
                }
                if (teamColor == TEAM_COLOR.BLUE) {
                    heading = SolveShootPoint.solveBLUEShootHeading(pose);
                    targetSpeed = calculateShootSpeed(SolveShootPoint.solveBLUEShootDistance(pose));
                }
                chassis.setHeadingLockRadian(heading);
            }
        }
        
        // 切换 DRL 模式
        if(gamepad2.bWasPressed()){
            switch (drlMode) {
                case NoDRL:
                    drlMode = DRL_MODE.FullDRL;
                    break;
                case FullDRL:
                    drlMode = DRL_MODE.RunDRL;
                    break;
                case RunDRL:
                    drlMode = DRL_MODE.NoDRL;
                    break;
            }
            
            // 更新 DRL 模式设置
            SolveShootPoint.setUseDRLModel(drlMode != DRL_MODE.NoDRL);
            ShooterAction.useDRL = drlMode != DRL_MODE.NoDRL;
            
            telemetry.addData("DRL Mode", drlMode.toString());
            telemetry.update();
            sleep(200);
        }

        if(ReadyToShoot){
            double currentHeading = chassis.robotPosition.getData().headingRadian;
            double targetHeading = chassis.getHeadingLockRadian();
            if(Math.abs(currentHeading - targetHeading) < startShootingHeading){
                robotStatus = ROBOT_STATUS.SHOOTING;
            }
        }
    }
    void setStatus(){
        switch (robotStatus) {
            case EATING:
                sweeperStatus = SWEEPER_STATUS.EAT;
                shooterStatus = SHOOTER_STATUS.STOP;
                triggerStatus = TRIGGER_STATUS.CLOSE;
                ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case WAITING:
                //boolean AprilTagStatus = !Double.isNaN(aprilTagDetector.getPose().pose.position.x);
                sweeperStatus = SWEEPER_STATUS.STOP;
                shooterStatus = SHOOTER_STATUS.STOP;
                triggerStatus = TRIGGER_STATUS.CLOSE;
                if(teamColor == TEAM_COLOR.RED){
//                    if(AprilTagStatus){
//                        ledController.setColor(LedPreset.HEARTBEAT_RED.getPattern());
//                    }
//                    else{
//                        ledController.showRedTeam();
//                    }
                    ledController.showRedTeam();
                }
                else{
//                    if(AprilTagStatus){
//                        ledController.setColor(LedPreset.HEARTBEAT_BLUE.getPattern());
//                    }
//                    else{
//                        ledController.showBlueTeam();
//                    }
                    ledController.showBlueTeam();
                }
                break;
            case SHOOTING:
                //ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                shooterStatus = SHOOTER_STATUS.SHOOTING;
                //sweeper和trigger状态由shooter条件决定，在shoot()中
                break;
            case OUTPUTTING:
                ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                sweeperStatus = SWEEPER_STATUS.OUTPUT;
                shooterStatus = SHOOTER_STATUS.STOP;
                triggerStatus = TRIGGER_STATUS.CLOSE;
                break;
            case CLIMBING:
                ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                sweeperStatus = SWEEPER_STATUS.STOP;
                shooterStatus = SHOOTER_STATUS.STOP;
                triggerStatus = TRIGGER_STATUS.CLOSE;
                if (gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                    directControl = true;
                    elevatorController.setPower(ElevatorController.BalancePower - gamepad2.left_trigger + gamepad2.right_trigger);
                } else {
                    if (directControl) {
                        directControl = false;
                        currentPosition = elevatorController.getPosition();
                    }
                    elevatorController.setPosition(currentPosition);
                }
                break;
        }

    }
    long lastSetTimeMS=0;
    boolean showSpeedColor=false;
    void Telemetry(){
        telemetry.addData("READYTOSHOOT", ReadyToShoot);
//        telemetry.addData("DIS", disSensor.getDis());
        telemetry.addData("Kspeed", Kspeed);
        telemetry.addData("RealTargetSpeed", targetSpeed * Kspeed);
        telemetry.addData("DegreeOffset", degreeOffset);
        telemetry.addData("NoHeadMode",chassis.getUseNoHeadMode()?"PlayerBased":"RoboticBased");
        telemetry.addData("isBusy", actionRunner.isBusy());
        telemetry.addData("TeamColor", teamColor);
        telemetry.addData("RobotSTATUS", robotStatus.toString());
        telemetry.addData("NoHeadModeStartError:",chassis.noHeadModeStartError);
        telemetry.addData("RunMode",chassis.runningToPoint?"RUNNING_TO_POINT":"MANUAL");
        telemetry.addData("DRL Mode", drlMode.toString());
        telemetry.addData("DRL Model Loaded", SolveShootPoint.isDRLModelLoaded() ? "Yes" : "No");
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
                    boolean hasReachedTargetSpeed = shooter.setShootSpeed(Math.toIntExact(Math.round(targetSpeed * Kspeed)));
                    double currentHeading = chassis.robotPosition.getData().headingRadian;
                    double targetHeading = chassis.getHeadingLockRadian();
                    boolean passShootCheckPoint = hasReachedTargetSpeed && Math.abs(currentHeading - targetHeading) < (toleranceHeading * KtoleranceHeading);
                    if(passShootCheckPoint){
                        sweeperStatus = SWEEPER_STATUS.GIVE_ARTIFACT;
                        triggerStatus = TRIGGER_STATUS.OPEN;
                    }
                    else{
                        //TODO 留下距离传感器判断是否有球的接口
                        if(targetSpeed * Kspeed > OpenSweeperSpeedThreshold){
                            sweeperStatus = SWEEPER_STATUS.STOP;
                        }
                    }

                    if(Math.abs(currentHeading - targetHeading) < toleranceHeading && hasReachedTargetSpeed){
                        ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    else{
                        ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                    }
                    if(gamepad2.startWasPressed()){
                        if(sweeperStatus == SWEEPER_STATUS.GIVE_ARTIFACT){
                            sweeperStatus = SWEEPER_STATUS.STOP;
                            triggerStatus = TRIGGER_STATUS.CLOSE;
                        }
                        if(sweeperStatus == SWEEPER_STATUS.STOP){
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
    public static double time_2=0.3;
    public static int IntervalMS=1;
    Pose2d pose = new Pose2d(0,0,0);
    /**
     * 底盘控制方法
     * 负责处理底盘的运动控制，包括：
     * 1. 设置自动航向锁定
     * 2. 更新机器人位置
     * 3. 处理游戏手柄输入
     * 4. 自动对准和目标点设置
     * 5. 无头模式切换
     */
    void chassis() {
        // 设置自动航向锁定（攀爬模式下不使用）
        chassis.setAutoLockHeading(robotStatus!=ROBOT_STATUS.CLIMBING);

        // 设置位置更新间隔
        chassis.robotPosition.setMinUpdateIntervalMs(IntervalMS);
        // 获取当前机器人位置
        pose = chassis.robotPosition.getData().getPose2d();

        // 计算驱动速度
        double drive = -time * gamepad1.left_stick_y - time_2 * gamepad2.left_stick_y; // 前后
        double strafe = time * gamepad1.left_stick_x + time_2 * gamepad2.left_stick_x; // 左右
        double rotate = -time * gamepad1.right_stick_x - time_2 * gamepad2.right_stick_x; // 旋转
        
        if(robotStatus!= ROBOT_STATUS.CLIMBING) {
            // 自动对准到射击角度
            if (gamepad1.left_trigger > 0.6) {
                double heading = 0;
                if (teamColor == TEAM_COLOR.RED) {
                    heading = SolveShootPoint.solveREDShootHeading(pose);
                    targetSpeed = calculateShootSpeed(SolveShootPoint.solveREDShootDistance(pose));
                }
                if (teamColor == TEAM_COLOR.BLUE) {
                    heading = SolveShootPoint.solveBLUEShootHeading(pose);
                    targetSpeed = calculateShootSpeed(SolveShootPoint.solveBLUEShootDistance(pose));
                }
                // 设置航向锁定角度
                chassis.setHeadingLockRadian(heading);
            }

            // 自动导航到射击位置
            if (gamepad1.dpadDownWasPressed()) {
                if (teamColor == TEAM_COLOR.RED) {
                    chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r4));
                    targetSpeed = ShooterAction.speed25_55;
                }
                if (teamColor == TEAM_COLOR.BLUE) {
                    chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r4));
                    targetSpeed = ShooterAction.speed25_55;
                }
            }

            if (gamepad1.dpadUpWasPressed()) {
                if (teamColor == TEAM_COLOR.RED) {
                    chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, SolveShootPoint.r5));
                    targetSpeed = ShooterAction.speed35_55;
                }
                if (teamColor == TEAM_COLOR.BLUE) {
                    chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, SolveShootPoint.r5));
                    targetSpeed = ShooterAction.speed35_55;
                }
            }
        }else{
            // 攀爬模式下导航到攀爬点
            if(gamepad2.rightStickButtonWasPressed() || gamepad2.leftStickButtonWasPressed()){
                chassis.setTargetPoint(SolveClimbPoint.solveClimbPoint(teamColor,pose));
            }
        }

        // 处理游戏手柄输入，控制机器人运动
        chassis.gamepadInput(strafe, drive, rotate);
        
        // 切换无头模式
        if(gamepad1.xWasReleased()) chassis.exchangeNoHeadMode();
        
        // 添加遥测数据
        telemetry.addData("y-power",drive);
        telemetry.addData("x-power",strafe);
        telemetry.addData("r-power",rotate);

        // 发送机器人位置到FtcDashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    void sweeper(){
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

    boolean InitStarted=false;
    @Override
    public void runOpMode() throws InterruptedException {
        //先初始化硬件
        Init();
        //留下更改一些参数的后门(??
        while(opModeInInit()||!InitStarted){
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
            telemetry.addData("Position(inch)", Point2D.rotate(chassis.robotPosition.getData().getPosition(DistanceUnit.INCH),teamColor==TEAM_COLOR.BLUE?Math.PI/2:-Math.PI/2).toString());
            telemetry.addData("TEAM_COLOR", teamColor.toString());
            telemetry.addData("FPS",1000000000.0/(System.nanoTime()-lastNanoTime));
            telemetry.update();
            InitStarted = true;
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
