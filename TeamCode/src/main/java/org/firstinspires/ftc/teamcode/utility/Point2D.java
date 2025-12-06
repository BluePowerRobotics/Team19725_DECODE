package org.firstinspires.ftc.teamcode.utility;

/**
 * Point2D类表示二维平面上的一个点，包含了点的坐标、极角和距离等属性。
 * 提供了一系列静态方法用于点的操作，如平移、旋转、缩放等。
 */
public class Point2D {
    /**
     * 点的x坐标
     */
    private double x;
    public double getX(){
        return x;
    }
    public void setX(double newX){
        x=newX;
    }
    /**
     * 点的y坐标
     */
    private double y;
    public double getY(){
        return y;
    }
    public void setY(double newY){
        y=newY;
    }


    public double getRadian(){
        return Math.atan2(y, x);
    }
    public double getDistance(){
        return Math.hypot(x,y);
    }

    /**
     * 构造函数，创建一个Point2D对象
     * @param x 点的x坐标
     * @param y 点的y坐标
     */
    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point2D(Point2D p){
        this(p.x,p.y);
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
    public static final Point2D ZERO = new Point2D(0, 0);
    /**
     * 用于判断是否为零的容差
     */
    public static double zeroTolerance = 1e-10;
    /**
     * 判断两个点是否相同
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 如果两个点的坐标差小于零容差，则认为它们是相同的
     */
    public static boolean equal(Point2D p1, Point2D p2) {
        return Math.abs(p1.x - p2.x) < zeroTolerance && Math.abs(p1.y - p2.y) < zeroTolerance;
    }
    /**
     * 计算两点之间的距离
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 两点之间的欧几里得距离
     */
    public static double getDistance(Point2D p1, Point2D p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    /**
     * 将点p平移offset
     * @param p 原始点
     * @param offset 平移偏移量
     * @return 平移后的新点
     */
    public static Point2D translate(Point2D p, Point2D offset) {
        return translate(p,offset.x,offset.y);
    }
    /**
     * 将点p平移dx和dy
     * @param p 原始点
     * @param dx x轴方向的平移量
     * @param dy y轴方向的平移量
     * @return 平移后的新点
     */
    public static Point2D translate(Point2D p, double dx, double dy) {
        return new Point2D(p.x + dx, p.y + dy);
    }
    /**
     * 将点p沿着指定的弧度和距离平移
     * @param p 原始点
     * @param Radian 平移的弧度
     * @param Distance 平移的距离
     * @return 平移后的新点
     */
    public static Point2D translateRD(Point2D p, double Radian, double Distance) {
        return new Point2D(p.x + Distance * Math.cos(Radian), p.y + Distance * Math.sin(Radian));
    }
    /**
     * 将点p绕原点沿着指定的弧度旋转
     * @param p 原始点
     * @param Radian 旋转的弧度
     * @return 旋转后的新点
     */
    public static Point2D rotate(Point2D p, double Radian) {
        return new Point2D(p.x * Math.cos(Radian) - p.y * Math.sin(Radian), p.x * Math.sin(Radian) + p.y * Math.cos(Radian));
    }
    /**
     * 将点p绕指定中心点旋转指定弧度
     * @param p 原始点
     * @param radian 旋转的弧度
     * @param center 旋转中心点
     * @return 旋转后的新点
     */
    public static Point2D rotate(Point2D p, double radian, Point2D center) {
        Point2D translated = translate(p, -center.x, -center.y);
        Point2D rotated = rotate(translated, radian);
        return translate(rotated, center.x, center.y);
    }
    /**
     * 计算两点的中点
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 两点的中点
     */
    public static Point2D getMidpoint(Point2D p1, Point2D p2) {
        return getMidpoint(p1.x,p1.y,p2.x,p2.y);
    }
    public static Point2D getMidpoint(double x1,double y1,double x2,double y2) {
        return new Point2D((x1 + x2) / 2, (y1 + y2) / 2);
    }
    /**
     * 缩放点p到指定的比例因子
     * @param p 原始点
     * @param factor 缩放因子
     * @return 缩放后的新点
     */
    public static Point2D scale(Point2D p, double factor) {
        return new Point2D(p.x * factor, p.y * factor);
    }
    /**
     * 缩放点p到指定的比例因子，绕指定中心点缩放
     * @param p 原始点
     * @param factor 缩放因子
     * @param center 缩放中心点
     * @return 缩放后的新点
     */
    public static Point2D scale(Point2D p, double factor, Point2D center) {
        Point2D translated = translate(p, -center.x, -center.y);
        Point2D scaled = scale(translated, factor);
        return translate(scaled, center.x, center.y);
    }
    /**
     * 从极坐标系转换为笛卡尔坐标系
     * @param radian 极角
     * @param distance 距离原点的距离
     * @return 笛卡尔坐标系中的点
     */
    public static Point2D fromPolar(double radian, double distance) {
        return new Point2D(distance * Math.cos(radian), distance * Math.sin(radian));
    }
    /**
     * 计算点p关于中心点center的中心对称点
     * @param p 原始点
     * @param center 中心点
     * @return 中心对称点
     */
    public static Point2D getCentralSymmetry(Point2D p, Point2D center) {
        return new Point2D(2 * center.x - p.x, 2 * center.y - p.y);
    }
    /**
     * 计算点p关于原点的中心对称点
     * @param p 原始点
     * @return 中心对称点
     */
    public static Point2D getCentralSymmetry(Point2D p) {
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
    public static Point2D getAxisSymmetry(Point2D p, double k, double b) {
        // 处理垂直线（斜率无限大）
        if (Double.isInfinite(k)) {
            return getAxisSymmetryVertical(p, b);
        }

        // 处理水平线（斜率为零）
        if (Math.abs(k) < zeroTolerance) {
            return getAxisSymmetryHorizontal(p, b);
        }

        // 处理普通斜线 y = kx + b
        return getAxisSymmetrySlant(p, k, b);
    }

    /**
     * 关于垂直线 x = c 的对称
     * @param p 原始点
     * @param x 垂直线的x坐标
     * @return 对称点
     */
    private static Point2D getAxisSymmetryVertical(Point2D p, double x) {
        return new Point2D(2 * x - p.x, p.y);
    }

    /**
     * 关于水平线 y = c 的对称
     * @param p 原始点
     * @param y 水平线的y坐标
     * @return 对称点
     */
    private static Point2D getAxisSymmetryHorizontal(Point2D p, double y) {
        return new Point2D(p.x, 2 * y - p.y);
    }

    /**
     * 关于斜线 y = kx + b 的对称
     * @param p 原始点
     * @param k 斜率
     * @param b 截距
     * @return 对称点
     */
    private static Point2D getAxisSymmetrySlant(Point2D p, double k, double b) {
        double k2 = k * k;
        double denominator = 1 + k2;

        double x1 = ((1 - k2) * p.x + 2 * k * (p.y - b)) / denominator;
        double y1 = (2 * k * p.x + (k2 - 1) * p.y + 2 * b) / denominator;

        return new Point2D(x1, y1);
    }
    /**
     * 计算两点的点积
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 两点的点积
     */
    public static double dot(Point2D p1, Point2D p2) {
        return p1.x * p2.x + p1.y * p2.y;
    }
    /**
     * 计算两点的叉积
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 两点的叉积，结果是一个新的Point2D对象，其x和y分别表示叉积的结果
     */
    public static Point2D cross(Point2D p1, Point2D p2) {
        return new Point2D(p1.x * p2.y - p1.y * p2.x, p1.y * p2.x - p1.x * p2.y);
    }
    /**
     * 转化为三维点（使用平面法向量和一个平面上的点计算平面位置，并将平面上的二维点转化为三维点）
     * @param p 原始二维点
     * @param planeNormal 平面法向量
     * @param planePoint 平面上的一个点
     * @return 三维点
     */
    public static Point3D toPoint3D(Point2D p, Point3D planeNormal, Point3D planePoint) {
        // 归一化法向量
        Point3D unitNormal = Point3D.normalize(planeNormal);

        // 找到平面上的两个正交基向量
        Point3D uAxis = findUAxis(unitNormal);
        Point3D vAxis = Point3D.cross(unitNormal, uAxis);

        // 将二维点坐标作为在平面基向量上的分量
        Point3D offset = Point3D.translateXYZ(
                Point3D.ZERO,
                p.x * uAxis.getX() + p.y * vAxis.getX(),
                p.x * uAxis.getY() + p.y * vAxis.getY(),
                p.x * uAxis.getZ() + p.y * vAxis.getZ()
        );

        // 从平面原点加上偏移量
        return Point3D.translate(planePoint, offset);
    }

    /**
     * 辅助方法：找到与法向量正交的U轴
     * 避免与法向量平行的情况
     */
    private static Point3D findUAxis(Point3D normal) {
        // 尝试使用X轴作为参考
        Point3D reference = new Point3D(1, 0, 0);

        // 如果法向量与X轴几乎平行，改用Y轴
        if (Math.abs(Point3D.dot(normal, reference)) > 0.9) {
            reference = new Point3D(0, 1, 0);
        }

        // 计算U轴 = reference - (reference·normal) * normal
        double dotProduct = Point3D.dot(reference, normal);
        return Point3D.normalize(
                new Point3D(
                        reference.getX() - dotProduct * normal.getX(),
                        reference.getY() - dotProduct * normal.getY(),
                        reference.getZ() - dotProduct * normal.getZ()
                )
        );
    }
}

