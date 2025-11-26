package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.RoadRunnerFileWriter;

@Autonomous
@Config
public class TEST_FileWriter extends LinearOpMode {
    public static double x = -64.8;
    public static double y = -17.6;
    public static double heading = 0;
    @Override
    public void runOpMode() {
        RoadRunnerFileWriter roadRunnerFileWriter = new RoadRunnerFileWriter();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(x,y,heading));
        Actions.runBlocking(
                roadRunnerFileWriter.WriteFile(drive)
        );
        telemetry.addData("test","file write complete");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            idle();
        }
    }
}
