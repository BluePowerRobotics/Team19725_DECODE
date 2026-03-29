package org.firstinspires.ftc.teamcode.Vision.model;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.RectF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class Dectector {
    public static String MODEL_NAME = "yolov8n.onnx";
    public static int INPUT_SIZE = 640;
    public static float CONFIDENCE_THRESHOLD = 0.5f;
    public static int resolutionwidth = 640;
    public static int resolutionheight = 480;
    public static String CAMERA_NAME = "Webcam 1";

    private Net net;
    private List<String> labels;

    VisionPortal portal;
    private CameraStreamProcessor processor;

    // 摄像头画面处理器，用于传输视频流到FTC dashboard
    public static class CameraStreamProcessor implements VisionProcessor {
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }

        public Bitmap getLastFrame() {
            return lastFrame.get();
        }
    }

    // 初始化检测器
    public void init(HardwareMap hardwareMap) {
        try {
            // 初始化OpenCV
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

            // 加载模型
            String modelPath = hardwareMap.appContext.getFilesDir().getAbsolutePath() + "/" + MODEL_NAME;
            
            // 从raw资源复制模型到应用目录
            try {
                int resourceId = hardwareMap.appContext.getResources().getIdentifier(MODEL_NAME.replace(".onnx", ""), "raw", hardwareMap.appContext.getPackageName());
                if (resourceId != 0) {
                    InputStream inputStream = hardwareMap.appContext.getResources().openRawResource(resourceId);
                    FileOutputStream outputStream = new FileOutputStream(modelPath);
                    
                    byte[] buffer = new byte[1024];
                    int bytesRead;
                    while ((bytesRead = inputStream.read(buffer)) != -1) {
                        outputStream.write(buffer, 0, bytesRead);
                    }
                    
                    inputStream.close();
                    outputStream.close();
                } else {
                    // 如果raw资源不存在，尝试从assets加载
                    try {
                        InputStream inputStream = hardwareMap.appContext.getAssets().open(MODEL_NAME);
                        FileOutputStream outputStream = new FileOutputStream(modelPath);
                        
                        byte[] buffer = new byte[1024];
                        int bytesRead;
                        while ((bytesRead = inputStream.read(buffer)) != -1) {
                            outputStream.write(buffer, 0, bytesRead);
                        }
                        
                        inputStream.close();
                        outputStream.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
            
            // 加载模型
            net = Dnn.readNetFromONNX(modelPath);

            // 加载标签（如果有）
            labels = new ArrayList<>();
            // 默认COCO数据集标签
            labels.add("person");
            labels.add("bicycle");
            labels.add("car");
            labels.add("motorcycle");
            labels.add("airplane");
            labels.add("bus");
            labels.add("train");
            labels.add("truck");
            labels.add("boat");
            labels.add("traffic light");
            labels.add("fire hydrant");
            labels.add("stop sign");
            labels.add("parking meter");
            labels.add("bench");
            labels.add("bird");
            labels.add("cat");
            labels.add("dog");
            labels.add("horse");
            labels.add("sheep");
            labels.add("cow");
            labels.add("elephant");
            labels.add("bear");
            labels.add("zebra");
            labels.add("giraffe");
            labels.add("backpack");
            labels.add("umbrella");
            labels.add("handbag");
            labels.add("tie");
            labels.add("suitcase");
            labels.add("frisbee");
            labels.add("skis");
            labels.add("snowboard");
            labels.add("sports ball");
            labels.add("kite");
            labels.add("baseball bat");
            labels.add("baseball glove");
            labels.add("skateboard");
            labels.add("surfboard");
            labels.add("tennis racket");
            labels.add("bottle");
            labels.add("wine glass");
            labels.add("cup");
            labels.add("fork");
            labels.add("knife");
            labels.add("spoon");
            labels.add("bowl");
            labels.add("banana");
            labels.add("apple");
            labels.add("sandwich");
            labels.add("orange");
            labels.add("broccoli");
            labels.add("carrot");
            labels.add("hot dog");
            labels.add("pizza");
            labels.add("donut");
            labels.add("cake");
            labels.add("chair");
            labels.add("couch");
            labels.add("potted plant");
            labels.add("bed");
            labels.add("dining table");
            labels.add("toilet");
            labels.add("tv");
            labels.add("laptop");
            labels.add("mouse");
            labels.add("remote");
            labels.add("keyboard");
            labels.add("cell phone");
            labels.add("microwave");
            labels.add("oven");
            labels.add("toaster");
            labels.add("sink");
            labels.add("refrigerator");
            labels.add("book");
            labels.add("clock");
            labels.add("vase");
            labels.add("scissors");
            labels.add("teddy bear");
            labels.add("hair drier");
            labels.add("toothbrush");

            // 初始化摄像头
            processor = new CameraStreamProcessor();
            portal = new VisionPortal.Builder()
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(processor)
                    .setCameraResolution(new android.util.Size(resolutionwidth, resolutionheight))
                    .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                    .build();

            // FtcDashboard.getInstance().startCameraStream(processor, 0);
            // 注意：CameraStreamProcessor未实现CameraStreamSource接口，暂时注释掉

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // 预测方法
    public List<DetectionResult> detect() {
        List<DetectionResult> results = new ArrayList<>();

        try {
            // 获取摄像头画面
            Bitmap bitmap = processor.getLastFrame();
            if (bitmap == null) {
                return results;
            }

            // 转换为Mat
            Mat frame = new Mat();
            Utils.bitmapToMat(bitmap, frame);

            // 预处理图像
            Mat blob = Dnn.blobFromImage(frame, 1.0 / 255.0, new Size(INPUT_SIZE, INPUT_SIZE), new Scalar(0, 0, 0), true, false);
            net.setInput(blob);

            // 运行模型
            Mat output = net.forward();

            // 后处理结果
            results = processOutput(output, frame.width(), frame.height());

            // 释放资源
            frame.release();
            blob.release();
            output.release();

        } catch (Exception e) {
            e.printStackTrace();
        }

        return results;
    }

    // 处理模型输出
    private List<DetectionResult> processOutput(Mat output, int imageWidth, int imageHeight) {
        List<DetectionResult> results = new ArrayList<>();

        // YOLOv8输出格式：[batch, num_detections, 7]，其中7包括：x, y, width, height, confidence, class_id, class_score
        int numDetections = output.size(1);

        for (int i = 0; i < numDetections; i++) {
            // 获取当前检测结果
            Mat detection = output.row(i);
            float[] data = new float[(int) detection.total()];
            detection.get(0, 0, data);

            float x = data[0];
            float y = data[1];
            float width = data[2];
            float height = data[3];
            float confidence = data[4];
            int classId = (int) data[5];

            if (confidence >= CONFIDENCE_THRESHOLD) {
                // 转换坐标到原始图像
                float left = (x - width / 2) * imageWidth;
                float top = (y - height / 2) * imageHeight;
                float right = (x + width / 2) * imageWidth;
                float bottom = (y + height / 2) * imageHeight;

                String label = classId < labels.size() ? labels.get(classId) : "unknown";
                results.add(new DetectionResult(new RectF(left, top, right, bottom), confidence, label));
            }

            // 释放资源
            detection.release();
        }

        return results;
    }

    // 释放资源
    public void close() {
        if (portal != null) {
            portal.close();
        }
    }

    // 检测结果类
    public static class DetectionResult {
        public RectF boundingBox;
        public float confidence;
        public String label;

        public DetectionResult(RectF boundingBox, float confidence, String label) {
            this.boundingBox = boundingBox;
            this.confidence = confidence;
            this.label = label;
        }
    }
}