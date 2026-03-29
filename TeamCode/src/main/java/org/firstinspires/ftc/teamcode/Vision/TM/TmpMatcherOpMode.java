package org.firstinspires.ftc.teamcode.Vision.TM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

@TeleOp(name = "TmpMatcher Test", group = "Vision")
public class TmpMatcherOpMode extends LinearOpMode {
    private OpenCvCamera camera;
    private TmpMatcher azqMatcher;
    private TmpMatcher fxcMatcher;
    private boolean isProcessing = false;

    @Override
    public void runOpMode() {
        // 初始化模板匹配器
        azqMatcher = new TmpMatcher("AZQ_template.JPG");
        fxcMatcher = new TmpMatcher("FXC_template.JPG");

        // 初始化相机
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // 设置相机回调
        camera.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                if (isProcessing) {
                    // 处理图像
                    azqMatcher.processFrame(input);
                    fxcMatcher.processFrame(input);
                }
                return input;
            }
        });

        // 启动相机
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 切换处理状态
            if (gamepad1.a) {
                isProcessing = !isProcessing;
                telemetry.addData("Processing", isProcessing);
                telemetry.update();
                sleep(500);
            }

            // 显示检测结果
            if (isProcessing) {
                List<TmpMatcher.TrackedObject> azqObjects = azqMatcher.getTrackedObjects();
                List<TmpMatcher.TrackedObject> fxcObjects = fxcMatcher.getTrackedObjects();

                telemetry.addData("AZQ Objects", azqObjects.size());
                for (TmpMatcher.TrackedObject obj : azqObjects) {
                    Rect rect = obj.getRect();
                    telemetry.addData("AZQ ID " + obj.getId(), "(" + rect.x + ", " + rect.y + ") " + rect.width + "x" + rect.height);
                }

                telemetry.addData("FXC Objects", fxcObjects.size());
                for (TmpMatcher.TrackedObject obj : fxcObjects) {
                    Rect rect = obj.getRect();
                    telemetry.addData("FXC ID " + obj.getId(), "(" + rect.x + ", " + rect.y + ") " + rect.width + "x" + rect.height);
                }
            }

            telemetry.update();
        }

        // 停止相机
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}