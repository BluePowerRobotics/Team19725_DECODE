package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Vision.TM.TmpMatcher;

@TeleOp(name = "TmpMatcher Test OpMode", group = "Test")
public class TmpMatcherTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing TmpMatcher...");
        telemetry.update();
        
        // 测试TmpMatcher构造函数
        try {
            TmpMatcher azqMatcher = new TmpMatcher("AZQ_template.JPG");
            TmpMatcher fxcMatcher = new TmpMatcher("FXC_template.JPG");
            
            telemetry.addData("TmpMatcher", "Created successfully");
            telemetry.addData("AZQ Matcher", "Initialized");
            telemetry.addData("FXC Matcher", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to create TmpMatcher: " + e.getMessage());
        }
        
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running TmpMatcher Test");
            telemetry.addData("Instructions", "Press A to test TmpMatcher");
            
            if (gamepad1.a) {
                telemetry.addData("Testing", "TmpMatcher methods");
                
                // 测试TrackedObject类
                try {
                    org.opencv.core.Rect rect = new org.opencv.core.Rect(100, 100, 50, 50);
                    TmpMatcher.TrackedObject object = new TmpMatcher.TrackedObject(rect, 1);
                    
                    telemetry.addData("TrackedObject", "Created successfully");
                    telemetry.addData("Object ID", object.getId());
                    telemetry.addData("Object Position", "(" + object.getRect().x + ", " + object.getRect().y + ")");
                    telemetry.addData("Object Size", object.getRect().width + "x" + object.getRect().height);
                    telemetry.addData("Missing Frames", object.getMissingFrames());
                    
                    // 测试update方法
                    org.opencv.core.Rect newRect = new org.opencv.core.Rect(110, 110, 50, 50);
                    object.update(newRect);
                    telemetry.addData("After Update", "Position: (" + object.getRect().x + ", " + object.getRect().y + ")");
                    
                    // 测试incrementMissingFrames方法
                    object.incrementMissingFrames();
                    telemetry.addData("Missing Frames", object.getMissingFrames());
                    
                    // 测试predict方法
                    object.predict();
                    telemetry.addData("Predict", "Executed successfully");
                    
                } catch (Exception e) {
                    telemetry.addData("Error", "TrackedObject test failed: " + e.getMessage());
                }
                
                sleep(1000);
            }
            
            telemetry.update();
        }
    }
}
