package org.firstinspires.ftc.teamcode.utility.filter;

import org.firstinspires.ftc.teamcode.utility.Point2D;

/**
 * 使用Point2D实现的移动平均角度滤波器
 * 利用向量运算避免角度跳变问题
 */
public class AngleMeanFilter {
    private final int windowSize;
    private final Point2D[] unitVectors;  // 存储单位向量
    private int index = 0;
    private int count = 0;
    private Point2D vectorSum = new Point2D(0, 0);  // 向量累加和

    public AngleMeanFilter(int windowSize) {
        if (windowSize <= 0) {
            throw new IllegalArgumentException("windowSize must be > 0");
        }
        this.windowSize = windowSize;
        this.unitVectors = new Point2D[windowSize];
        for (int i = 0; i < windowSize; i++) {
            unitVectors[i] = new Point2D(0, 0);
        }
    }

    /**
     * 添加一个新角度（弧度）并返回平均角度（弧度）
     */
    public double filter(double angleRad) {
        // 将角度转换为单位向量
        Point2D newVector = Point2D.fromPolar(angleRad, 1.0);

        if (count < windowSize) {
            // 缓冲区未满
            unitVectors[index] = newVector;
            vectorSum = Point2D.translate(vectorSum, newVector.getX(), newVector.getY());
            count++;
            index = (index + 1) % windowSize;
        } else {
            // 缓冲区已满，替换最旧向量
            Point2D oldest = unitVectors[index];
            vectorSum = Point2D.translate(vectorSum, -oldest.getX(), -oldest.getY());

            unitVectors[index] = newVector;
            vectorSum = Point2D.translate(vectorSum, newVector.getX(), newVector.getY());

            index = (index + 1) % windowSize;
        }

        // 计算平均向量的角度
        return vectorSum.getRadian();
    }

    /**
     * 添加新角度（度数）并返回平均角度（度数）
     */
    public double filterDegrees(double angleDeg) {
        double angleRad = Math.toRadians(angleDeg);
        double resultRad = filter(angleRad);
        return Math.toDegrees(resultRad);
    }

    /**
     * 重置滤波器
     */
    public void reset() {
        for (int i = 0; i < windowSize; i++) {
            unitVectors[i].setX(0);
            unitVectors[i].setY(0);
        }
        vectorSum.setX(0);
        vectorSum.setY(0);
        index = 0;
        count = 0;
    }

    /**
     * 获取当前平均角度的向量长度（表示滤波器一致性）
     * 值越接近1，表示角度分布越集中
     */
    public double getConsistency() {
        if (count == 0) return 0;
        return vectorSum.getDistance() / Math.min(count, windowSize);
    }
}