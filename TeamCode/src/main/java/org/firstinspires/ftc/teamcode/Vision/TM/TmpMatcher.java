package org.firstinspires.ftc.teamcode.Vision.TM;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.*;

public class TmpMatcher {
    private static final int MAX_MISSING_FRAMES = 5;
    private static final double NMS_IOU_THRESHOLD = 0.3;
    private static final double MATCH_THRESHOLD = 0.8;
    private static final double SCALE_MIN = 0.5;
    private static final double SCALE_MAX = 2.0;
    private static final double SCALE_STEP = 0.2;
    
    private Mat template;
    private String templateName;
    private List<TrackedObject> trackedObjects;
    private int nextObjectId;
    
    public TmpMatcher(String templateName) {
        this.templateName = templateName;
        this.trackedObjects = new ArrayList<>();
        this.nextObjectId = 0;
        
        // 加载模板图像
        String templatePath = "Vision/TM/template/" + templateName;
        this.template = Imgcodecs.imread(templatePath);
        if (template.empty()) {
            System.err.println("Failed to load template: " + templatePath);
            return;
        }
    }
    
    public void processFrame(Mat frame) {
        if (template.empty()) {
            return;
        }
        
        // 检测目标
        List<Rect> detections = detect(frame);
        
        // 关联检测结果与跟踪对象
        associateDetectionsWithTrackers(detections);
        
        // 更新跟踪对象
        updateTrackers();
        
        // 非极大值抑制
        List<TrackedObject> filteredObjects = applyNMS();
        
        // 绘制结果
        drawResults(frame, filteredObjects);
    }
    
    private List<Rect> detect(Mat frame) {
        List<Rect> detections = new ArrayList<>();
        
        // 转换为灰度图像
        Mat grayFrame = new Mat();
        Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_BGR2GRAY);
        
        // 多尺度检测
        for (double scale = SCALE_MIN; scale <= SCALE_MAX; scale += SCALE_STEP) {
            // 缩放模板
            Mat resizedTemplate = new Mat();
            Imgproc.resize(template, resizedTemplate, new Size(), scale, scale);
            
            // 确保模板小于图像
            if (resizedTemplate.cols() > frame.cols() || resizedTemplate.rows() > frame.rows()) {
                resizedTemplate.release();
                continue;
            }
            
            // 转换缩放模板为灰度
            Mat grayTemplate = new Mat();
            Imgproc.cvtColor(resizedTemplate, grayTemplate, Imgproc.COLOR_BGR2GRAY);
            
            // 创建结果矩阵
            int resultCols = grayFrame.cols() - grayTemplate.cols() + 1;
            int resultRows = grayFrame.rows() - grayTemplate.rows() + 1;
            Mat result = new Mat(resultRows, resultCols, CvType.CV_32FC1);
            
            // 执行模板匹配
            Imgproc.matchTemplate(grayFrame, grayTemplate, result, Imgproc.TM_CCOEFF_NORMED);
            
            // 查找所有大于阈值的匹配
            Mat mask = new Mat();
            Imgproc.threshold(result, mask, MATCH_THRESHOLD, 1.0, Imgproc.THRESH_BINARY);
            
            // 查找所有匹配位置
            while (true) {
                Core.MinMaxLocResult mmr = Core.minMaxLoc(mask);
                if (mmr.maxVal > 0) {
                    Point matchLoc = mmr.maxLoc;
                    Rect rect = new Rect((int)matchLoc.x, (int)matchLoc.y, grayTemplate.cols(), grayTemplate.rows());
                    detections.add(rect);
                    
                    // 在掩码中标记已检测的区域，避免重复检测
                    Imgproc.rectangle(mask, matchLoc, 
                        new Point(matchLoc.x + grayTemplate.cols(), matchLoc.y + grayTemplate.rows()), 
                        new Scalar(0), -1);
                } else {
                    break;
                }
            }
            
            // 释放资源
            resizedTemplate.release();
            grayTemplate.release();
            result.release();
            mask.release();
        }
        
        // 释放资源
        grayFrame.release();
        
        return detections;
    }
    
    private void associateDetectionsWithTrackers(List<Rect> detections) {
        if (trackedObjects.isEmpty()) {
            // 没有跟踪对象，创建新的
            for (Rect detection : detections) {
                TrackedObject obj = new TrackedObject(detection, nextObjectId++);
                trackedObjects.add(obj);
            }
            return;
        }
        
        // 计算IOU矩阵
        double[][] iouMatrix = new double[trackedObjects.size()][detections.size()];
        for (int i = 0; i < trackedObjects.size(); i++) {
            for (int j = 0; j < detections.size(); j++) {
                iouMatrix[i][j] = calculateIOU(trackedObjects.get(i).getRect(), detections.get(j));
            }
        }
        
        // 贪心匹配
        Set<Integer> matchedTrackers = new HashSet<>();
        Set<Integer> matchedDetections = new HashSet<>();
        
        // 按IOU排序
        List<Match> matches = new ArrayList<>();
        for (int i = 0; i < trackedObjects.size(); i++) {
            for (int j = 0; j < detections.size(); j++) {
                if (iouMatrix[i][j] > 0.1) { // 最小IOU阈值
                    matches.add(new Match(i, j, iouMatrix[i][j]));
                }
            }
        }
        
        // 按IOU降序排序
        matches.sort((m1, m2) -> Double.compare(m2.iou, m1.iou));
        
        // 匹配
        for (Match match : matches) {
            if (!matchedTrackers.contains(match.trackerIdx) && !matchedDetections.contains(match.detectionIdx)) {
                // 更新跟踪对象
                TrackedObject obj = trackedObjects.get(match.trackerIdx);
                obj.update(detections.get(match.detectionIdx));
                matchedTrackers.add(match.trackerIdx);
                matchedDetections.add(match.detectionIdx);
            }
        }
        
        // 处理未匹配的跟踪对象
        for (int i = 0; i < trackedObjects.size(); i++) {
            if (!matchedTrackers.contains(i)) {
                TrackedObject obj = trackedObjects.get(i);
                obj.incrementMissingFrames();
            }
        }
        
        // 处理未匹配的检测结果
        for (int j = 0; j < detections.size(); j++) {
            if (!matchedDetections.contains(j)) {
                TrackedObject obj = new TrackedObject(detections.get(j), nextObjectId++);
                trackedObjects.add(obj);
            }
        }
        
        // 移除丢失超过阈值的跟踪对象
        trackedObjects.removeIf(obj -> obj.getMissingFrames() > MAX_MISSING_FRAMES);
    }
    
    private void updateTrackers() {
        for (TrackedObject obj : trackedObjects) {
            obj.predict();
        }
    }
    
    private List<TrackedObject> applyNMS() {
        List<TrackedObject> result = new ArrayList<>();
        List<TrackedObject> objects = new ArrayList<>(trackedObjects);
        
        // 按面积降序排序
        objects.sort((o1, o2) -> Double.compare(o2.getRect().area(), o1.getRect().area()));
        
        while (!objects.isEmpty()) {
            TrackedObject current = objects.remove(0);
            result.add(current);
            
            objects.removeIf(obj -> calculateIOU(current.getRect(), obj.getRect()) > NMS_IOU_THRESHOLD);
        }
        
        return result;
    }
    
    private void drawResults(Mat frame, List<TrackedObject> objects) {
        for (TrackedObject obj : objects) {
            Rect rect = obj.getRect();
            Imgproc.rectangle(frame, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), 
                             new Scalar(0, 255, 0), 2);
            Imgproc.putText(frame, "ID: " + obj.getId(), new Point(rect.x, rect.y - 10), 
                          Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
            Imgproc.putText(frame, "Size: " + rect.width + "x" + rect.height, 
                          new Point(rect.x, rect.y + rect.height + 20), 
                          Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
        }
    }
    
    private double calculateIOU(Rect a, Rect b) {
        int x1 = Math.max(a.x, b.x);
        int y1 = Math.max(a.y, b.y);
        int x2 = Math.min(a.x + a.width, b.x + b.width);
        int y2 = Math.min(a.y + a.height, b.y + b.height);
        
        int intersection = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
        int union = (int)(a.area() + b.area() - intersection);
        
        return union > 0 ? (double) intersection / union : 0;
    }
    
    public List<TrackedObject> getTrackedObjects() {
        return trackedObjects;
    }
    
    // 内部类：跟踪对象
    public static class TrackedObject {
        private Rect rect;
        private int id;
        private int missingFrames;
        
        public TrackedObject(Rect rect, int id) {
            this.rect = rect;
            this.id = id;
            this.missingFrames = 0;
        }
        
        public void update(Rect newRect) {
            this.rect = newRect;
            this.missingFrames = 0;
        }
        
        public void predict() {
            // 简单的预测：保持当前位置
        }
        
        public void incrementMissingFrames() {
            missingFrames++;
        }
        
        public Rect getRect() {
            return rect;
        }
        
        public int getId() {
            return id;
        }
        
        public int getMissingFrames() {
            return missingFrames;
        }
    }
    
    // 内部类：匹配
    private static class Match {
        int trackerIdx;
        int detectionIdx;
        double iou;
        
        Match(int trackerIdx, int detectionIdx, double iou) {
            this.trackerIdx = trackerIdx;
            this.detectionIdx = detectionIdx;
            this.iou = iou;
        }
    }
}
