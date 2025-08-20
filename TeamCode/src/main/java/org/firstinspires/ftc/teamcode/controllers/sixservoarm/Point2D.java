package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

/**
 * Point2D类表示二维平面上的一个点，包含了点的坐标、极角和距离等属性。
 * 提供了一系列静态方法用于点的操作，如平移、旋转、缩放等。
 */
public class Point2D {
    /**
     * 点的x坐标
     */
    public double x;
    /**
     * 点的y坐标
     */
    public double y;
    /**
     * 点的极角（弧度）
     * 计算方式：Math.atan2(y, x)
     */
    public double Radian;
    /**
     * 点到原点的距离
     * 计算方式：Math.sqrt(x * x + y * y)
     */
    public double Distance;

    /**
     * 构造函数，创建一个Point2D对象
     * @param x 点的x坐标
     * @param y 点的y坐标
     */
    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
        this.Radian = Math.atan2(y, x); // 计算点的极角
        this.Distance = Math.sqrt(x * x + y * y); // 计算点到原点的距离
    }

    /**
     * 转化为字符串表示形式
     * @return 字符串形式的点坐标
     */
    @Override
    public String toString() {
        return "( " + x +
                " , " + y +
                " )";
    }

    /**
     * 零点坐标
     */
    static Point2D ZERO = new Point2D(0, 0);
    /**
     * 用于判断是否为零的容差
     */
    static double zeroTolerance = 1e-10;
    /**
     * 判断两个点是否相同
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 如果两个点的坐标差小于零容差，则认为它们是相同的
     */
    static boolean isSame(Point2D p1, Point2D p2) {
        return Math.abs(p1.x - p2.x) < zeroTolerance && Math.abs(p1.y - p2.y) < zeroTolerance;
    }
    /**
     * 计算两点之间的距离
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 两点之间的欧几里得距离
     */
    static double distance(Point2D p1, Point2D p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    /**
     * 将点p平移offset
     * @param p 原始点
     * @param offset 平移偏移量
     * @return 平移后的新点
     */
    static Point2D translate(Point2D p, Point2D offset) {
        return new Point2D(p.x + offset.x, p.y + offset.y);
    }
    /**
     * 将点p平移dx和dy
     * @param p 原始点
     * @param dx x轴方向的平移量
     * @param dy y轴方向的平移量
     * @return 平移后的新点
     */
    static Point2D translateXY(Point2D p, double dx, double dy) {
        return new Point2D(p.x + dx, p.y + dy);
    }
    /**
     * 将点p沿着指定的弧度和距离平移
     * @param p 原始点
     * @param Radian 平移的弧度
     * @param Distance 平移的距离
     * @return 平移后的新点
     */
    static Point2D translateRD(Point2D p, double Radian, double Distance) {
        return new Point2D(p.x + Distance * Math.cos(Radian), p.y + Distance * Math.sin(Radian));
    }
    /**
     * 将点p沿着指定的弧度平移，绕原点旋转
     * @param p 原始点
     * @param Radian 平移的弧度
     * @return 平移后的新点
     */
    static Point2D rotate(Point2D p, double Radian) {
        return new Point2D(p.x * Math.cos(Radian) - p.y * Math.sin(Radian), p.x * Math.sin(Radian) + p.y * Math.cos(Radian));
    }
    /**
     * 将点p绕指定中心点旋转指定弧度
     * @param p 原始点
     * @param Radian 旋转的弧度
     * @param center 旋转中心点
     * @return 旋转后的新点
     */
    static Point2D rotate(Point2D p, double Radian, Point2D center) {
        Point2D translated = translateXY(p, -center.x, -center.y);
        Point2D rotated = rotate(translated, Radian);
        return translateXY(rotated, center.x, center.y);
    }
    /**
     * 计算两点的中点
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 两点的中点
     */
    static Point2D midpoint(Point2D p1, Point2D p2) {
        return new Point2D((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    }
    /**
     * 缩放点p到指定的比例因子
     * @param p 原始点
     * @param factor 缩放因子
     * @return 缩放后的新点
     */
    static Point2D scale(Point2D p, double factor) {
        return new Point2D(p.x * factor, p.y * factor);
    }
    /**
     * 缩放点p到指定的比例因子，绕指定中心点缩放
     * @param p 原始点
     * @param factor 缩放因子
     * @param center 缩放中心点
     * @return 缩放后的新点
     */
    static Point2D scale(Point2D p, double factor, Point2D center) {
        Point2D translated = translateXY(p, -center.x, -center.y);
        Point2D scaled = scale(translated, factor);
        return translateXY(scaled, center.x, center.y);
    }
    /**
     * 从极坐标系转换为笛卡尔坐标系
     * @param Radian 极角
     * @param Distance 距离原点的距离
     * @return 笛卡尔坐标系中的点
     */
    static Point2D fromPolar(double Radian, double Distance) {
        return new Point2D(Distance * Math.cos(Radian), Distance * Math.sin(Radian));
    }
    /**
     * 计算点p关于中心点center的中心对称点
     * @param p 原始点
     * @param center 中心点
     * @return 中心对称点
     */
    static Point2D centralSymmetry(Point2D p, Point2D center) {
        return new Point2D(2 * center.x - p.x, 2 * center.y - p.y);
    }
    /**
     * 计算点p关于原点的中心对称点
     * @param p 原始点
     * @return 中心对称点
     */
    static Point2D centralSymmetry(Point2D p) {
        return new Point2D(-p.x, -p.y);
    }
    /**
     * 计算点关于直线的对称点（支持任意直线）
     * <p>
     * 直线表示方式：y = kx + b
     * <p>
     * 特殊直线处理：
     * 1. 水平线 (k=0): y = b <p>
     * 2. 垂直线 (k=∞): 将b解释为x坐标，即 x = b <p>
     * 3. 普通斜线: y = kx + b <p>
     * <p>
     * 使用示例： <p>
     * // 关于x轴对称 (y=0) <p>
     * axisSymmetry(point, 0, 0);
     * <p>
     * // 关于y轴对称 (x=0)   <p>
     * axisSymmetry(point, Double.POSITIVE_INFINITY, 0);
     * <p>
     * // 关于直线x=3对称 <p>
     * axisSymmetry(point, Double.POSITIVE_INFINITY, 3);
     * <p>
     * // 关于直线y=2对称 <p>
     * axisSymmetry(point, 0, 2);
     * <p>
     * // 关于斜线y=2x+1对称 <p>
     * axisSymmetry(point, 2, 1);
     *
     * @param p 原始点
     * @param k 直线斜率
     * @param b 直线截距（对于垂直线，b表示x坐标）
     * @return 对称点
     */
    static Point2D axisSymmetry(Point2D p, double k, double b) {
        // 处理垂直线（斜率无限大）
        if (Double.isInfinite(k)) {
            return axisSymmetryVertical(p, b);
        }

        // 处理水平线（斜率为零）
        if (Math.abs(k) < zeroTolerance) {
            return axisSymmetryHorizontal(p, b);
        }

        // 处理普通斜线 y = kx + b
        return axisSymmetrySlant(p, k, b);
    }

    /**
     * 关于垂直线 x = c 的对称
     * @param p 原始点
     * @param x 垂直线的x坐标
     * @return 对称点
     */
    private static Point2D axisSymmetryVertical(Point2D p, double x) {
        return new Point2D(2 * x - p.x, p.y);
    }

    /**
     * 关于水平线 y = c 的对称
     * @param p 原始点
     * @param y 水平线的y坐标
     * @return 对称点
     */
    private static Point2D axisSymmetryHorizontal(Point2D p, double y) {
        return new Point2D(p.x, 2 * y - p.y);
    }

    /**
     * 关于斜线 y = kx + b 的对称
     * @param p 原始点
     * @param k 斜率
     * @param b 截距
     * @return 对称点
     */
    private static Point2D axisSymmetrySlant(Point2D p, double k, double b) {
        double k2 = k * k;
        double denominator = 1 + k2;

        double x1 = ((1 - k2) * p.x + 2 * k * (p.y - b)) / denominator;
        double y1 = (2 * k * p.x + (k2 - 1) * p.y + 2 * b) / denominator;

        return new Point2D(x1, y1);
    }
}
