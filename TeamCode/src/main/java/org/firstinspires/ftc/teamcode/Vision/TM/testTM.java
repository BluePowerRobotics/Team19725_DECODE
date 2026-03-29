package org.firstinspires.ftc.teamcode.Vision.TM;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class testTM {
    public static void main(String[] args) {
        // 加载OpenCV库
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        
        // 初始化两个TmpMatcher对象
        TmpMatcher azqMatcher = new TmpMatcher("AZQ_template.JPG");
        TmpMatcher fxcMatcher = new TmpMatcher("FXC_template.JPG");
        
        // 测试图像路径
        String[] testImages = {
            "Vision/TM/test_data/DJI_20251009153125_0056_D.JPG",
            "Vision/TM/test_data/DJI_20251009153128_0057_D.JPG",
            "Vision/TM/test_data/DJI_20251009153137_0058_D.JPG"
        };
        
        // 处理每个测试图像
        for (String imagePath : testImages) {
            System.out.println("Processing image: " + imagePath);
            
            // 读取图像
            Mat frame = Imgcodecs.imread(imagePath);
            if (frame.empty()) {
                System.err.println("Failed to load image: " + imagePath);
                continue;
            }
            
            // 复制原始图像用于显示
            Mat resultFrame = frame.clone();
            
            // 使用两个匹配器处理图像
            azqMatcher.processFrame(resultFrame);
            fxcMatcher.processFrame(resultFrame);
            
            // 保存结果图像
            String outputPath = "Vision/TM/test_data/output_" + imagePath.substring(imagePath.lastIndexOf("/") + 1);
            Imgcodecs.imwrite(outputPath, resultFrame);
            System.out.println("Output saved to: " + outputPath);
            
            // 输出检测结果
            List<TmpMatcher.TrackedObject> azqObjects = azqMatcher.getTrackedObjects();
            List<TmpMatcher.TrackedObject> fxcObjects = fxcMatcher.getTrackedObjects();
            
            System.out.println("AZQ objects detected: " + azqObjects.size());
            for (TmpMatcher.TrackedObject obj : azqObjects) {
                Rect rect = obj.getRect();
                System.out.println("  ID: " + obj.getId() + ", Position: (" + rect.x + ", " + rect.y + "), Size: " + rect.width + "x" + rect.height);
            }
            
            System.out.println("FXC objects detected: " + fxcObjects.size());
            for (TmpMatcher.TrackedObject obj : fxcObjects) {
                Rect rect = obj.getRect();
                System.out.println("  ID: " + obj.getId() + ", Position: (" + rect.x + ", " + rect.y + "), Size: " + rect.width + "x" + rect.height);
            }
            
            System.out.println();
            
            // 释放资源
            frame.release();
            resultFrame.release();
        }
        
        System.out.println("Test completed!");
    }
}
