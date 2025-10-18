package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
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



//泵赛季主程序
@TeleOp(name="DECODE", group="OpModes")
public class DECODE extends LinearOpMode {
    public enum TEAMCOLOR{
        RED,BLUE
    }
    TEAMCOLOR teamcolor;
    public enum TRIGGERSTATUS{
        OPEN,
        CLOSE,
        EMERGENCYSTOP
    }
    TRIGGERSTATUS triggerStatus = TRIGGERSTATUS.CLOSE;
    public enum SWEEPERSTATUS{
        EAT,
        GIVEARTIFACT,
        OUTPUT,
        STOP
    }
    SWEEPERSTATUS sweeperStatus = SWEEPERSTATUS.STOP;
    public enum SHOOTERSTATUS{
        SHOOTING,
        EMERGENCYSTOP,
        STOP
    }
    SHOOTERSTATUS shooterStatus = SHOOTERSTATUS.STOP;
    public ChassisController chassis;
    public Sweeper sweeper;
    public ShooterAction shooter;
    public Trigger trigger;
    //
    public  int targetSpeed = 900;
    public static int speed2_2 = 900;
    public static int speed25_25 = 925;
    public static int speed3_3 = 975;
    public static int speed25_55 = 1100;
    public static int speed35_55 = 1230;
    public static double r1 = 48 * Math.sqrt(2);
    public static double r2 = 60 * Math.sqrt(2);
    public static double r3 = 72 * Math.sqrt(2);
    //特殊情况，表示小三角，靠近球门一边射击
    public static double r4 = 100;
    //特殊情况，表示小三角，远离球门一边射击
    public static double r5 = 200;
    public Pose2d startPose = new Pose2d(0,0,0);
    void Init(){

        //todo set team color
        teamcolor = TEAMCOLOR.BLUE;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sweeper = new Sweeper(hardwareMap);
        trigger = new Trigger(hardwareMap);
        shooter = new ShooterAction(hardwareMap, telemetry);
        chassis = new ChassisController(hardwareMap, new Point2D(0,0), 0);
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
            case OPEN:
                trigger.open();
                break;
            case CLOSE:
                trigger.close();
                break;
            case EMERGENCYSTOP:
                trigger.emergencyStop();
                break;
        }
    }

    void shoot(){

        if(gamepad1.yWasPressed() || gamepad2.aWasPressed()){
            if(shooterStatus == SHOOTERSTATUS.SHOOTING){
                shooterStatus = SHOOTERSTATUS.EMERGENCYSTOP;
            }
            else{
                shooterStatus = SHOOTERSTATUS.SHOOTING;
            }
        }

        if(gamepad1.aWasPressed() || gamepad2.aWasPressed()){
            shooterStatus = SHOOTERSTATUS.STOP;
        }


        switch (shooterStatus){
            case SHOOTING:
                boolean ifhit =   false;//todo = chassisController.wheelSpeeds.length;
                if(ifhit){
                    shooterStatus = SHOOTERSTATUS.EMERGENCYSTOP;
                }
                else{
                    sweeperStatus = SWEEPERSTATUS.GIVEARTIFACT;
                    triggerStatus = TRIGGERSTATUS.OPEN;
                    shooter.setShootSpeed(targetSpeed);
                }
                break;

            case EMERGENCYSTOP:
                triggerStatus = TRIGGERSTATUS.EMERGENCYSTOP;
                sweeperStatus = SWEEPERSTATUS.STOP;
                shooter.setShootSpeed(-400);
                break;

            case STOP:
                triggerStatus = TRIGGERSTATUS.CLOSE;
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
            if(teamcolor == TEAMCOLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, r1));
                targetSpeed = speed2_2;
            }
            if(teamcolor == TEAMCOLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, r1));
                targetSpeed = speed2_2;
            }
        }
        if(gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()){
            if(teamcolor == TEAMCOLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, r2));
                targetSpeed = speed25_25;
            }
            if(teamcolor == TEAMCOLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, r2));
                targetSpeed = speed25_25;
            }
        }
        if(gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()){
            if(teamcolor == TEAMCOLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, r3));
                targetSpeed = speed3_3;
            }
            if(teamcolor == TEAMCOLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, r3));
                targetSpeed = speed3_3;
            }
        }
        if(gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()){
            if(teamcolor == TEAMCOLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, r4));
                targetSpeed = speed25_55;
            }
            if(teamcolor == TEAMCOLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, r4));
                targetSpeed = speed25_55;
            }
        }
        if(gamepad2.xWasPressed()){
            if(teamcolor == TEAMCOLOR.RED){
                chassis.setTargetPoint(SolveShootPoint.solveREDShootPoint(pose, r5));
                targetSpeed = speed35_55;
            }
            if(teamcolor == TEAMCOLOR.BLUE){
                chassis.setTargetPoint(SolveShootPoint.solveBLUEShootPoint(pose, r5));
                targetSpeed = speed35_55;
            }
        }


        chassis.gamepadInput(strafe, drive, rotate);
        if(gamepad1.xWasReleased()) chassis.exchangeNoHeadMode();
        if(gamepad1.yWasReleased()) {
            //chassis.setTargetPoint(new Pose2d(new Vector2d(0,0), Rotation2d.fromDouble(0)));
        }
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
            if(sweeperStatus == SWEEPERSTATUS.EAT){
                sweeperStatus = SWEEPERSTATUS.STOP;
            }
            else{
                sweeperStatus = SWEEPERSTATUS.EAT;
            }

        }
        else if(gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()){
            if(sweeperStatus == SWEEPERSTATUS.OUTPUT){
                sweeperStatus = SWEEPERSTATUS.STOP;
            }
            else{
                sweeperStatus = SWEEPERSTATUS.OUTPUT;
            }
        }

        if(gamepad2.yWasPressed()){
            sweeperStatus = SWEEPERSTATUS.GIVEARTIFACT;
        }

        switch (sweeperStatus){
            case EAT:
                sweeper.Eat();
                break;
            case GIVEARTIFACT:
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
