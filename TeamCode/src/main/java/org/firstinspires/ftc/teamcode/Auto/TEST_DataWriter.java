package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.chassis.locate.Data;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
@TeleOp(name = "TEST_DataWriter", group = "TEST")
public class TEST_DataWriter extends LinearOpMode {

    public void initiate(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);
        Data.getInstance().setPosition(new Point2D(0,0));
        Data.getInstance().headingRadian=0;

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initiate();
        waitForStart();
        last_time_ms=now_time_ms=System.currentTimeMillis();
        while (opModeIsActive()){
            addTele();
            setLocate();

        }
        Data.getInstance().setPose2d(new Pose2d(Point.getY(),-Point.getX(),HeadingRadian));
    }
    public void addTele(){
        last_time_ms=now_time_ms;
        now_time_ms=System.currentTimeMillis();
        telemetry.addData("FPS",1000.0/(now_time_ms-last_time_ms));
        telemetry.addLine();
        telemetry.addData("Point", Point.toString());
        telemetry.addData("HeadingRadian", HeadingRadian);
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(Point.getY(),-Point.getX(), HeadingRadian));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    public void setLocate(){
        if(now_time_ms-last_set_time_ms>50){
            Point=Point2D.translate(Point,new Point2D(2*gamepad1.left_stick_x,-2*gamepad1.left_stick_y));

            HeadingRadian+=gamepad1.right_stick_x*0.1;
            HeadingRadian= MathSolver.normalizeAngle(HeadingRadian);
            last_set_time_ms=now_time_ms;
        }
    }

    public long now_time_ms,last_time_ms,last_set_time_ms;
    Point2D Point = new Point2D(0,0);
    double HeadingRadian =0;
}
