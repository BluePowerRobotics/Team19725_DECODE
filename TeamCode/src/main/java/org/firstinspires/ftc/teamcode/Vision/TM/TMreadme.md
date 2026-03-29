# 模板匹配器使用说明

本文档详细说明了 FTC 机器人项目中使用的两个模板匹配器：`TmpMatcher`（单模板匹配）和 `MultiTmpMatcher`（多模板匹配）。

## 目录
- [TmpMatcher - 单模板匹配器](#tmpmatcher---单模板匹配器)
- [MultiTmpMatcher - 多模板匹配器](#multitmpmatcher---多模板匹配器)
- [使用方法](#使用方法)
- [测试方法](#测试方法)
- [参数调优指南](#参数调优指南)
- [常见问题与解决方案](#常见问题与解决方案)

---

## TmpMatcher - 单模板匹配器

### 功能概述
`TmpMatcher` 是一个基于 OpenCV 的单模板匹配器，用于在图像中检测和跟踪特定模板目标。

### 核心功能
1. **模板匹配**
   - 使用 OpenCV 的 `matchTemplate` 函数进行模板匹配
   - 采用 `TM_CCOEFF_NORMED` 方法，归一化相关系数匹配
   - 支持多尺度检测（0.5x 到 2.0x，步长 0.2）

2. **目标跟踪**
   - 使用 IOU（交并比）进行检测结果与跟踪对象的关联
   - 贪心匹配算法确保最佳匹配
   - 支持多目标同时跟踪

3. **非极大值抑制（NMS）**
   - 消除重叠的检测结果
   - IOU 阈值：0.3
   - 按检测框面积降序排序

4. **丢失目标处理**
   - 最大丢失帧数：5 帧
   - 超过阈值自动移除跟踪对象

### 关键参数
```java
private static final int MAX_MISSING_FRAMES = 5;        // 最大丢失帧数
private static final double NMS_IOU_THRESHOLD = 0.3;   // NMS IOU 阈值
private static final double MATCH_THRESHOLD = 0.8;      // 匹配阈值
private static final double SCALE_MIN = 0.5;            // 最小缩放比例
private static final double SCALE_MAX = 2.0;            // 最大缩放比例
private static final double SCALE_STEP = 0.2;           // 缩放步长
```

### API 接口
```java
// 构造函数
public TmpMatcher(String templateName)

// 处理帧
public void processFrame(Mat frame)

// 获取跟踪对象
public List<TrackedObject> getTrackedObjects()

// TrackedObject 方法
public Rect getRect()           // 获取检测框
public int getId()               // 获取对象 ID
public int getMissingFrames()    // 获取丢失帧数
```

---

## MultiTmpMatcher - 多模板匹配器

### 功能概述
`MultiTmpMatcher` 扩展了单模板匹配功能，支持同时识别多个不同类型的模板目标。

### 核心功能
1. **多模板支持**
   - 支持加载多个模板图像
   - 每个检测结果记录对应的模板类型
   - 可动态添加模板（通过构造函数）

2. **模板类型识别**
   - 每个跟踪对象记录模板索引和名称
   - 可视化显示模板类型信息
   - 支持不同类型目标的独立跟踪

3. **增强的检测功能**
   - 对每个模板独立进行多尺度检测
   - 所有模板的检测结果统一处理
   - 保持与单模板版本相同的跟踪和 NMS 逻辑

4. **可视化增强**
   - 显示对象 ID
   - 显示模板类型名称
   - 显示检测框尺寸

### 关键参数
与 `TmpMatcher` 相同的参数配置，确保一致的性能表现。

### API 接口
```java
// 构造函数 - 单模板
public MultiTmpMatcher(String templateName)

// 构造函数 - 多模板
public MultiTmpMatcher(List<String> templateNames)

// 处理帧
public void processFrame(Mat frame)

// 获取跟踪对象
public List<TrackedObject> getTrackedObjects()

// TrackedObject 方法
public Rect getRect()           // 获取检测框
public int getId()               // 获取对象 ID
public int getMissingFrames()    // 获取丢失帧数
public int getTemplateIdx()      // 获取模板索引
public String getTemplateName() // 获取模板名称
```

---

## 使用方法

### 1. 准备模板图像

将模板图像放置在项目目录：
```
TeamCode/src/main/assets/Vision/TM/template/
```

支持的图像格式：PNG、JPG、JPEG

**模板图像建议：**
- 分辨率：建议 100x100 到 300x300 像素
- 对比度：高对比度特征更易识别
- 背景：建议使用透明背景或纯色背景
- 特征：包含明显的边缘和纹理特征

### 2. TmpMatcher 使用示例

```java
import org.firstinspires.ftc.teamcode.Vision.TM.TmpMatcher;
import org.opencv.core.Mat;

public class SingleTemplateExample {
    private TmpMatcher matcher;
    
    public void init() {
        // 初始化匹配器
        matcher = new TmpMatcher("red_marker.png");
    }
    
    public void processFrame(Mat frame) {
        // 处理帧
        matcher.processFrame(frame);
        
        // 获取检测结果
        List<TmpMatcher.TrackedObject> objects = matcher.getTrackedObjects();
        
        for (TmpMatcher.TrackedObject obj : objects) {
            Rect rect = obj.getRect();
            int id = obj.getId();
            
            // 计算中心点
            double centerX = rect.x + rect.width / 2.0;
            double centerY = rect.y + rect.height / 2.0;
            
            System.out.println("Object ID: " + id);
            System.out.println("Position: (" + centerX + ", " + centerY + ")");
            System.out.println("Size: " + rect.width + "x" + rect.height);
        }
    }
}
```

### 3. MultiTmpMatcher 使用示例

```java
import org.firstinspires.ftc.teamcode.Vision.TM.MultiTmpMatcher;
import org.opencv.core.Mat;
import java.util.Arrays;
import java.util.List;

public class MultiTemplateExample {
    private MultiTmpMatcher matcher;
    
    public void init() {
        // 初始化多模板匹配器
        List<String> templates = Arrays.asList(
            "red_marker.png",
            "blue_marker.png",
            "yellow_marker.png"
        );
        matcher = new MultiTmpMatcher(templates);
    }
    
    public void processFrame(Mat frame) {
        // 处理帧
        matcher.processFrame(frame);
        
        // 获取检测结果
        List<MultiTmpMatcher.TrackedObject> objects = matcher.getTrackedObjects();
        
        for (MultiTmpMatcher.TrackedObject obj : objects) {
            Rect rect = obj.getRect();
            int id = obj.getId();
            String type = obj.getTemplateName();
            
            // 计算中心点
            double centerX = rect.x + rect.width / 2.0;
            double centerY = rect.y + rect.height / 2.0;
            
            System.out.println("Object ID: " + id);
            System.out.println("Type: " + type);
            System.out.println("Position: (" + centerX + ", " + centerY + ")");
            System.out.println("Size: " + rect.width + "x" + rect.height);
            
            // 根据类型执行不同操作
            if (type.contains("red")) {
                handleRedMarker(centerX, centerY);
            } else if (type.contains("blue")) {
                handleBlueMarker(centerX, centerY);
            } else if (type.contains("yellow")) {
                handleYellowMarker(centerX, centerY);
            }
        }
    }
    
    private void handleRedMarker(double x, double y) {
        // 处理红色标记
    }
    
    private void handleBlueMarker(double x, double y) {
        // 处理蓝色标记
    }
    
    private void handleYellowMarker(double x, double y) {
        // 处理黄色标记
    }
}
```

### 4. FTC 集成示例

```java
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.TM.MultiTmpMatcher;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Template Matching Auto")
public class TemplateMatchingAuto extends LinearOpMode {
    private OpenCvCamera camera;
    private MultiTmpMatcher matcher;
    
    @Override
    public void runOpMode() {
        // 初始化摄像头
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        // 初始化匹配器
        matcher = new MultiTmpMatcher(Arrays.asList(
            "red_pixel.png",
            "blue_pixel.png",
            "green_pixel.png"
        ));
        
        // 启动摄像头
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
        
        // 设置处理管道
        camera.setPipeline(new TemplateMatchingPipeline());
        
        waitForStart();
        
        // 自动阶段逻辑
        while (opModeIsActive()) {
            List<MultiTmpMatcher.TrackedObject> objects = matcher.getTrackedObjects();
            
            if (!objects.isEmpty()) {
                MultiTmpMatcher.TrackedObject obj = objects.get(0);
                Rect rect = obj.getRect();
                double centerX = rect.x + rect.width / 2.0;
                
                telemetry.addData("Detected", obj.getTemplateName());
                telemetry.addData("Position", centerX);
                telemetry.update();
                
                // 根据检测结果执行机器人动作
                if (centerX < 200) {
                    moveLeft();
                } else if (centerX > 440) {
                    moveRight();
                } else {
                    moveForward();
                }
            }
        }
    }
    
    class TemplateMatchingPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            matcher.processFrame(input);
            return input;
        }
    }
    
    private void moveLeft() {
        // 向左移动逻辑
    }
    
    private void moveRight() {
        // 向右移动逻辑
    }
    
    private void moveForward() {
        // 向前移动逻辑
    }
}
```

---

## 测试方法

### 1. 单元测试

创建测试类验证基本功能：

```java
import org.junit.Test;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import static org.junit.Assert.*;

public class TmpMatcherTest {
    
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    
    @Test
    public void testSingleTemplateMatching() {
        // 加载测试图像
        Mat testFrame = Imgcodecs.imread("test_images/test_frame.jpg");
        assertNotNull("Test frame not loaded", testFrame);
        
        // 创建匹配器
        TmpMatcher matcher = new TmpMatcher("test_template.png");
        
        // 处理帧
        matcher.processFrame(testFrame);
        
        // 验证结果
        List<TmpMatcher.TrackedObject> objects = matcher.getTrackedObjects();
        assertNotNull("Objects list should not be null", objects);
        
        // 释放资源
        testFrame.release();
    }
    
    @Test
    public void testMultiTemplateMatching() {
        Mat testFrame = Imgcodecs.imread("test_images/test_frame.jpg");
        assertNotNull("Test frame not loaded", testFrame);
        
        List<String> templates = Arrays.asList(
            "template1.png",
            "template2.png",
            "template3.png"
        );
        
        MultiTmpMatcher matcher = new MultiTmpMatcher(templates);
        matcher.processFrame(testFrame);
        
        List<MultiTmpMatcher.TrackedObject> objects = matcher.getTrackedObjects();
        assertNotNull("Objects list should not be null", objects);
        
        testFrame.release();
    }
}
```

### 2. 可视化测试

使用 EasyOpenCV 进行实时可视化测试：

```java
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisualTest {
    private OpenCvCamera camera;
    private TmpMatcher matcher;
    
    public void runVisualTest() {
        // 初始化摄像头
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        // 初始化匹配器
        matcher = new TmpMatcher("test_template.png");
        
        // 启动摄像头
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            
            @Override
            public void onError(int errorCode) {
                System.err.println("Camera Error: " + errorCode);
            }
        });
        
        // 设置处理管道
        camera.setPipeline(new TestPipeline());
        
        // 观察检测结果
        while (true) {
            List<TmpMatcher.TrackedObject> objects = matcher.getTrackedObjects();
            System.out.println("Detected objects: " + objects.size());
            
            for (TmpMatcher.TrackedObject obj : objects) {
                System.out.println("ID: " + obj.getId() + 
                                 ", Rect: " + obj.getRect());
            }
            
            sleep(100);
        }
    }
    
    class TestPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            matcher.processFrame(input);
            return input;
        }
    }
}
```

### 3. 性能测试

测试处理速度和准确率：

```java
public class PerformanceTest {
    public void runPerformanceTest() {
        MultiTmpMatcher matcher = new MultiTmpMatcher(Arrays.asList(
            "template1.png", "template2.png", "template3.png"
        ));
        
        Mat testFrame = Imgcodecs.imread("test_images/test_frame.jpg");
        
        // 预热
        for (int i = 0; i < 10; i++) {
            matcher.processFrame(testFrame);
        }
        
        // 性能测试
        long startTime = System.currentTimeMillis();
        int iterations = 100;
        
        for (int i = 0; i < iterations; i++) {
            matcher.processFrame(testFrame);
        }
        
        long endTime = System.currentTimeMillis();
        double avgTime = (endTime - startTime) / (double) iterations;
        
        System.out.println("Average processing time: " + avgTime + " ms");
        System.out.println("FPS: " + (1000.0 / avgTime));
        
        testFrame.release();
    }
}
```

### 4. 准确率测试

使用标注数据集测试检测准确率：

```java
public class AccuracyTest {
    public void runAccuracyTest() {
        MultiTmpMatcher matcher = new MultiTmpMatcher(Arrays.asList(
            "template1.png", "template2.png"
        ));
        
        int totalTests = 0;
        int correctDetections = 0;
        
        // 测试图像列表
        List<String> testImages = Arrays.asList(
            "test_images/image1.jpg",
            "test_images/image2.jpg",
            "test_images/image3.jpg"
        );
        
        for (String imagePath : testImages) {
            Mat frame = Imgcodecs.imread(imagePath);
            matcher.processFrame(frame);
            
            List<MultiTmpMatcher.TrackedObject> objects = matcher.getTrackedObjects();
            
            // 根据标注验证检测结果
            if (verifyDetection(objects, imagePath)) {
                correctDetections++;
            }
            
            totalTests++;
            frame.release();
        }
        
        double accuracy = (double) correctDetections / totalTests * 100;
        System.out.println("Accuracy: " + accuracy + "%");
    }
    
    private boolean verifyDetection(List<MultiTmpMatcher.TrackedObject> objects, 
                                    String imagePath) {
        // 实现验证逻辑
        // 可以与标注数据比较位置、类型等
        return !objects.isEmpty();
    }
}
```

---

## 参数调优指南

### 1. 匹配阈值（MATCH_THRESHOLD）

**默认值：** 0.8

**调整建议：**
- **提高阈值（0.85-0.95）**：减少误检，但可能漏检
- **降低阈值（0.7-0.8）**：增加检测率，但可能增加误检

**调优方法：**
```java
// 在类中修改
private static final double MATCH_THRESHOLD = 0.85;
```

### 2. NMS IOU 阈值（NMS_IOU_THRESHOLD）

**默认值：** 0.3

**调整建议：**
- **提高阈值（0.4-0.5）**：保留更多重叠检测
- **降低阈值（0.2-0.3）**：更严格地消除重叠

### 3. 缩放范围（SCALE_MIN, SCALE_MAX）

**默认值：** 0.5 - 2.0

**调整建议：**
- **已知目标大小范围**：根据实际目标调整
- **目标较小**：降低 SCALE_MIN 到 0.3
- **目标较大**：提高 SCALE_MAX 到 3.0

### 4. 最大丢失帧数（MAX_MISSING_FRAMES）

**默认值：** 5

**调整建议：**
- **快速移动目标**：降低到 3-4
- **稳定目标**：提高到 7-10

---

## 常见问题与解决方案

### 1. 无法检测到目标

**可能原因：**
- 模板图像不合适
- 匹配阈值过高
- 缩放范围不合适

**解决方案：**
- 优化模板图像（提高对比度、减少背景）
- 降低 MATCH_THRESHOLD 到 0.7-0.75
- 扩大缩放范围（SCALE_MIN: 0.3, SCALE_MAX: 2.5）

### 2. 误检过多

**可能原因：**
- 匹配阈值过低
- 模板特征不明显
- 背景干扰

**解决方案：**
- 提高 MATCH_THRESHOLD 到 0.85-0.9
- 优化模板图像
- 降低 NMS_IOU_THRESHOLD 到 0.2

### 3. 检测速度慢

**可能原因：**
- 缩放步长过小
- 模板数量过多
- 图像分辨率过高

**解决方案：**
- 增大 SCALE_STEP 到 0.3-0.4
- 减少模板数量
- 降低摄像头分辨率（640x480 或 320x240）

### 4. 目标跟踪不稳定

**可能原因：**
- 目标移动过快
- 丢失帧数设置不合理
- IOU 阈值过低

**解决方案：**
- 提高 MAX_MISSING_FRAMES 到 7-10
- 提高关联 IOU 阈值到 0.15-0.2
- 优化 predict() 方法实现运动预测

### 5. 多模板混淆

**可能原因：**
- 模板之间相似度过高
- 匹配阈值设置不当

**解决方案：**
- 确保模板之间有明显差异
- 为不同类型模板设置不同的匹配阈值
- 使用颜色预处理增强区分度

---

## FTC 应用建议

### 1. 自动阶段（Autonomous）

**推荐使用：** `MultiTmpMatcher`

**应用场景：**
- 识别不同颜色的游戏元素
- 多目标同时检测和定位
- 基于检测结果进行路径规划

**最佳实践：**
```java
// 在自动阶段使用多模板匹配
List<String> autoTemplates = Arrays.asList(
    "red_spike.png",
    "blue_spike.png",
    "backdrop_target.png"
);
MultiTmpMatcher autoMatcher = new MultiTmpMatcher(autoTemplates);
```

### 2. 手动阶段（TeleOp）

**推荐使用：** `TmpMatcher` 或 `MultiTmpMatcher`

**应用场景：**
- 实时目标检测辅助操作
- 多目标状态监控
- 自动瞄准辅助

**最佳实践：**
```java
// 在手动阶段使用单模板匹配以减少计算负担
TmpMatcher teleMatcher = new TmpMatcher("target.png");
```

### 3. 性能优化建议

1. **降低分辨率**：使用 640x480 或 320x240
2. **减少模板数量**：只加载必要的模板
3. **增大缩放步长**：SCALE_STEP = 0.3
4. **限制检测区域**：只处理感兴趣区域（ROI）

---

## 总结

- **TmpMatcher**：适合单目标检测，计算效率高
- **MultiTmpMatcher**：适合多目标检测，功能更强大
- 根据具体应用场景选择合适的匹配器
- 通过参数调优获得最佳性能
- 充分测试确保可靠性

如有问题，请参考 FTC 官方文档或联系团队技术支持。
