package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Vision.model.AprilTagInfo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class AprilTagDetector {
    //todo 可以参考的文件：FindCandidate.java  && FtcRobotController\src\main\java\org\firstinspires\ftc\robotcontroller\external\samples\ConceptAprilTagLocalization.java
    //画面大小
    public static double x = 0;
    public static double y = -6.22;
    public static double z = 10.83;
    public static double yaw = 180;
    public static double pitch = 73.6;
    public static double roll = 0;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            x, y, z, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            yaw, pitch, roll, 0);
    public static int resolutionwidth = 640;
    public static int resolutionheight= 480;
    VisionPortal portal;
    private AprilTagProcessor aprilTag;


    //一个简单的摄像头画面处理器（processor），可以附加到vision portal上
    // 功能：传输摄像头视频流到FTC dashboard
    //todo ：不用做修改，传输视频的功能已经实现
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }@Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }@Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {

        }@Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
    public enum MOTIFTYPE{
        PPG, PGP, GPP, UNKNOWN
    }


    //todo 在这里进行初始化
    public void init(HardwareMap hardWareMap){
        CameraStreamProcessor processor = new CameraStreamProcessor();

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        portal = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .addProcessor(processor)
                .setCameraResolution(new Size(resolutionwidth, resolutionheight))
                .setCamera(hardWareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        FtcDashboard.getInstance().startCameraStream(processor, 0);
    }


    //todo 我会在auto开始调用，进行识别，根据识别到的编号返回对应MOTIFTYPE
    public MOTIFTYPE decodeAprilTag(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                int id = detection.id;
                if(id == 23){
                    return MOTIFTYPE.PPG;
                } else if(id == 22){
                    return MOTIFTYPE.PGP;
                } else if(id == 21){
                    return MOTIFTYPE.GPP;
                }
            }
        }
        return MOTIFTYPE.UNKNOWN;
    }

    //todo 会在每一帧调用，在这里识别AprilTag并返回位姿 **只识别两个球门上的AprilTag，排除编号为21，22，23的AprilTag**
    //返回值为空数组表示未识别到AprilTag
    //如识别到多个AprilTag，返回数组中包含多个AprilTagInfo
    public AprilTagInfo getPose(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        ArrayList<AprilTagInfo> tagInfos = new ArrayList<>();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id != 21 && detection.id != 22 && detection.id != 23) {

                Pose2d pose = new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES))+Math.PI/2);

                AprilTagInfo info = new AprilTagInfo(
                        pose,
                        detection.id,
                        detection.ftcPose.range,
                        detection.ftcPose.pitch
                );

                tagInfos.add(info);
            }
        }

        // 修改：如果未识别到任何AprilTag，返回一个包含全为NaN的AprilTagInfo对象
        if (tagInfos.isEmpty()) {
            AprilTagInfo invalidInfo = new AprilTagInfo(
                    new Pose2d(Double.NaN, Double.NaN, Double.NaN),
                    -1,
                    Double.NaN,
                    Double.NaN
            );
            tagInfos.add(invalidInfo);
        }

        return tagInfos.get(0);
    }
}