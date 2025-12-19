package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Config
public class TEST_FileWriter extends LinearOpMode {
    //public static double x = -64.8;
    public static double x = 0;
    //public static double y = -17.6;
    public static double y = 0;
    public static double heading = 0;
    @Override
    public void runOpMode() {
        try (java.io.FileWriter writer = new java.io.FileWriter("/sdcard/FIRST/pose.txt")) {
            // 这里假设 drive 和 localizer 已经初始化，并且有 getPose() 方法
            // 示例数据，实际应替换为你的 drive.localizer.getPose()
            writer.write(x + "," + y + "," + heading);
        } catch (java.io.IOException e) {
            throw new RuntimeException(e);
        }
        telemetry.addData("test","file write complete");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            idle();
        }
    }
}

