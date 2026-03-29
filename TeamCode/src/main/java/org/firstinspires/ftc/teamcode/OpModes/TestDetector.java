package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.model.Dectector;

import java.util.List;

@TeleOp(name = "Test Detector", group = "Vision")
public class TestDetector extends LinearOpMode {
    private Dectector detector;

    @Override
    public void runOpMode() {
        // 初始化检测器
        detector = new Dectector();
        detector.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "Initializing...");
        telemetry.update();

        // 等待摄像头启动
        sleep(2000);
        telemetry.addData("Status", "Ready");
        telemetry.addData("Camera", "Started");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 运行检测
            List<Dectector.DetectionResult> results = detector.detect();

            // 显示检测结果
            telemetry.addData("Status", "Running");
            telemetry.addData("Detected objects", results.size());
            for (int i = 0; i < results.size(); i++) {
                Dectector.DetectionResult result = results.get(i);
                telemetry.addData("Object " + i, result.label + " (" + String.format("%.2f", result.confidence) + ")");
                telemetry.addData("  Position", "X: " + String.format("%.2f", result.boundingBox.centerX()) + ", Y: " + String.format("%.2f", result.boundingBox.centerY()));
                telemetry.addData("  Size", "W: " + String.format("%.2f", result.boundingBox.width()) + ", H: " + String.format("%.2f", result.boundingBox.height()));
            }

            telemetry.update();
        }

        // 释放资源
        detector.close();
    }
}