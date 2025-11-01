package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TEST_FileWriter extends LinearOpMode {
    @Override
    public void runOpMode() {
        try (java.io.FileWriter writer = new java.io.FileWriter("/sdcard/FIRST/pose.txt")) {
            // 这里假设 drive 和 localizer 已经初始化，并且有 getPose() 方法
            // 示例数据，实际应替换为你的 drive.localizer.getPose()
            double x = 1.23;
            double y = 4.56;
            double heading = 0.78;
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
