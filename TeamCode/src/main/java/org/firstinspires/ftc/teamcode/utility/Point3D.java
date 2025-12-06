package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

public class Point3D {
    /**
     * 点的x坐标
     */
    private double x;

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    /**
     * 点的y坐标
     */
    private double y;

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    /**
     * 点的z坐标
     */
    private double z;
    public double getZ() {
        return z;
    }
    public void setZ(double z) {
        this.z = z;
    }
    public double getDistance() {
        return Math.sqrt(x * x + y * y + z * z);
    }
    public double getAzimuth() {
        return Math.atan2(y, x);
    }
    public double getPolar() {
        return (getDistance() > zeroTolerance) ? Math.acos(z / getDistance()) : 0;
    }

    /**
     * 点坐标加法
     * @param other 另一个点
     */
    public void add(Point3D other) {
        this.x += other.x;
        this.y += other.y;
        this.z += other.z;
    }

    /**
     * 范围限制（长方体）
     * @param min 最小点坐标
     * @param max 最大点坐标
     */
    public void clamp(@NonNull Point3D min, @NonNull Point3D max) {
        this.x = Math.max(min.x, Math.min(max.x, this.x));
        this.y = Math.max(min.y, Math.min(max.y, this.y));
        this.z = Math.max(min.z, Math.min(max.z, this.z));
    }

    /**
     * 范围限制（球体）
     * @param maxDistance 最大距离
     * @param p 参考点
     */
    public void clamp(double maxDistance, @NonNull Point3D p) {
        double dist = distance(this, p);
        if (dist > maxDistance) {
            Point3D direction = translate(this, centralSymmetry(p));// 计算从p指向当前点的方向向量
            direction = normalize(direction);// 归一化方向向量
            // 将点移动到距离p为maxDistance的位置
            this.x = p.x + direction.x * maxDistance;
            this.y = p.y + direction.y * maxDistance;
            this.z = p.z + direction.z * maxDistance;
        }
    }

    /**
     * 判断是否为0
     * @return 如果点的坐标都小于零容差，则认为它是零点
     */
    boolean isZero() {
        return Math.abs(x) < zeroTolerance &&
                Math.abs(y) < zeroTolerance &&
                Math.abs(z) < zeroTolerance;
    }
    /**
     * 构造函数，创建一个Point3D对象
     * @param x 点的x坐标
     * @param y 点的y坐标
     * @param z 点的z坐标
     */
    public Point3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    /**
     * 复制数据，创建一个Point3D对象
     * @param other 另一个Point3D对象
     */
    public Point3D(Point3D other) {
        this(other.x, other.y, other.z);
    }

    /**
     * 转化为字符串表示形式
     * @return 字符串形式的点坐标
     */
    @Override
    public String toString() {
        return "( " + x + " , " + y + " , " + z + " )";
    }

    /**
     * 零点坐标
     */
    public static Point3D ZERO = new Point3D(0, 0, 0);
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
    public static boolean equal(Point3D p1, Point3D p2) {
        return Math.abs(p1.x - p2.x) < zeroTolerance &&
                Math.abs(p1.y - p2.y) < zeroTolerance &&
                Math.abs(p1.z - p2.z) < zeroTolerance;
    }

    /**
     * 计算两点之间的距离
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 两点之间的欧几里得距离
     */
    public static double distance(Point3D p1, Point3D p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) +
                Math.pow(p1.y - p2.y, 2) +
                Math.pow(p1.z - p2.z, 2));
    }

    /**
     * 向量点积（内积）
     * @param p1 第一个向量
     * @param p2 第二个向量
     * @return 点积结果
     */
    public static double dot(Point3D p1, Point3D p2) {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
    }

    /**
     * 向量叉积（外积）
     * @param p1 第一个向量
     * @param p2 第二个向量
     * @return 叉积结果向量
     */
    public static Point3D cross(Point3D p1, Point3D p2) {
        return new Point3D(
                p1.y * p2.z - p1.z * p2.y,
                p1.z * p2.x - p1.x * p2.z,
                p1.x * p2.y - p1.y * p2.x
        );
    }

    /**
     * 向量模长
     * @param p 向量
     * @return 向量的模长
     */
    public static double magnitude(Point3D p) {
        return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    /**
     * 向量归一化(单位向量)
     * @param p 向量
     * @return 单位向量
     */
    public static Point3D normalize(Point3D p) {
        double mag = magnitude(p);
        if (mag < zeroTolerance) return ZERO;
        return new Point3D(p.x / mag, p.y / mag, p.z / mag);
    }

    /**
     * 将点p平移offset
     * @param p 原始点
     * @param offset 平移偏移量
     * @return 平移后的新点
     */
    public static Point3D translate(Point3D p, Point3D offset) {
        return new Point3D(p.x + offset.x, p.y + offset.y, p.z + offset.z);
    }

    /**
     * 将点p平移dx, dy, dz
     * @param p 原始点
     * @param dx x轴方向的平移量
     * @param dy y轴方向的平移量
     * @param dz z轴方向的平移量
     * @return 平移后的新点
     */
    public static Point3D translateXYZ(Point3D p, double dx, double dy, double dz) {
        return new Point3D(p.x + dx, p.y + dy, p.z + dz);
    }

    /**
     * 绕X轴旋转
     * @param p 原始点
     * @param angle 旋转角度（弧度）
     * @return 旋转后的点
     */
    public static Point3D rotateX(Point3D p, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Point3D(
                p.x,
                p.y * cosA - p.z * sinA,
                p.y * sinA + p.z * cosA
        );
    }

    /**
     * 绕Y轴旋转
     * @param p 原始点
     * @param angle 旋转角度（弧度）
     * @return 旋转后的点
     */
    public static Point3D rotateY(Point3D p, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Point3D(
                p.x * cosA + p.z * sinA,
                p.y,
                -p.x * sinA + p.z * cosA
        );
    }

    /**
     * 绕Z轴旋转
     * @param p 原始点
     * @param angle 旋转角度（弧度）
     * @return 旋转后的点
     */
    public static Point3D rotateZ(Point3D p, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Point3D(
                p.x * cosA - p.y * sinA,
                p.x * sinA + p.y * cosA,
                p.z
        );
    }

    /**
     * 绕任意轴旋转（罗德里格斯旋转公式）
     * @param p 原始点
     * @param axis 旋转轴（单位向量）
     * @param angle 旋转角度（弧度）
     * @return 旋转后的点
     */
    public static Point3D rotateAroundAxis(Point3D p, Point3D axis, double angle) {
        Point3D unitAxis = normalize(axis);
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);

        // 罗德里格斯公式
        Point3D cross = cross(unitAxis, p);
        double dot = dot(unitAxis, p);

        return new Point3D(
                p.x * cosA + cross.x * sinA + unitAxis.x * dot * (1 - cosA),
                p.y * cosA + cross.y * sinA + unitAxis.y * dot * (1 - cosA),
                p.z * cosA + cross.z * sinA + unitAxis.z * dot * (1 - cosA)
        );
    }

    /**
     * 计算两点的中点
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 两点的中点
     */
    public static Point3D midpoint(Point3D p1, Point3D p2) {
        return new Point3D(
                (p1.x + p2.x) / 2,
                (p1.y + p2.y) / 2,
                (p1.z + p2.z) / 2
        );
    }

    /**
     * 缩放点p到指定的比例因子
     * @param p 原始点
     * @param factor 缩放因子
     * @return 缩放后的新点
     */
    public static Point3D scale(Point3D p, double factor) {
        return new Point3D(p.x * factor, p.y * factor, p.z * factor);
    }

    /**
     * 缩放点p到指定的比例因子，绕指定中心点缩放
     * @param p 原始点
     * @param factor 缩放因子
     * @param center 缩放中心点
     * @return 缩放后的新点
     */
    public static Point3D scale(Point3D p, double factor, Point3D center) {
        Point3D translated = translateXYZ(p, -center.x, -center.y, -center.z);
        Point3D scaled = scale(translated, factor);
        return translateXYZ(scaled, center.x, center.y, center.z);
    }

    /**
     * 从球坐标系转换为笛卡尔坐标系
     * @param azimuth 方位角（弧度）
     * @param polar 极角（弧度）
     * @param distance 距离
     * @return 笛卡尔坐标系中的点
     */
    public static Point3D fromSpherical(double azimuth, double polar, double distance) {
        double sinPolar = Math.sin(polar);
        return new Point3D(
                distance * sinPolar * Math.cos(azimuth),
                distance * sinPolar * Math.sin(azimuth),
                distance * Math.cos(polar)
        );
    }

    /**
     * 计算点p关于中心点center的中心对称点
     * @param p 原始点
     * @param center 中心点
     * @return 中心对称点
     */
    public static Point3D centralSymmetry(Point3D p, Point3D center) {
        return new Point3D(
                2 * center.x - p.x,
                2 * center.y - p.y,
                2 * center.z - p.z
        );
    }

    /**
     * 计算点p关于原点的中心对称点
     * @param p 原始点
     * @return 中心对称点
     */
    public static Point3D centralSymmetry(Point3D p) {
        return new Point3D(-p.x, -p.y, -p.z);
    }

    /**
     * 关于XY平面的对称
     * @param p 原始点
     * @return 对称点
     */
    public static Point3D symmetryAboutXYPlane(Point3D p) {
        return new Point3D(p.x, p.y, -p.z);
    }

    /**
     * 关于YZ平面的对称
     * @param p 原始点
     * @return 对称点
     */
    public static Point3D symmetryAboutYZPlane(Point3D p) {
        return new Point3D(-p.x, p.y, p.z);
    }

    /**
     * 关于XZ平面的对称
     * @param p 原始点
     * @return 对称点
     */
    public static Point3D symmetryAboutXZPlane(Point3D p) {
        return new Point3D(p.x, -p.y, p.z);
    }

    /**
     * 计算点到平面的距离
     * @param point 点
     * @param planeNormal 平面法向量（单位向量）
     * @param planePoint 平面上的一点
     * @return 点到平面的距离
     */
    public static double distanceToPlane(Point3D point, Point3D planeNormal, Point3D planePoint) {
        Point3D diff = translate(point, centralSymmetry(planePoint));
        return Math.abs(dot(diff, planeNormal));
    }

    /**
     * 计算点在平面上的投影
     * @param point 点
     * @param planeNormal 平面法向量（单位向量）
     * @param planePoint 平面上的一点
     * @return 点在平面上的投影
     */
    public static Point3D projectToPlane(Point3D point, Point3D planeNormal, Point3D planePoint) {
        Point3D diff = translate(point, centralSymmetry(planePoint));
        double distance = dot(diff, planeNormal);
        return translate(point, scale(planeNormal, -distance));
    }
    /**
     * 计算点对于任意平面的对称点
     * @param point 点
     * @param planeNormal 平面法向量（单位向量）
     * @param planePoint 平面上的一点
     * @return 点对于平面的对称点
     */
    public static Point3D symmetryAboutPlane(Point3D point, Point3D planeNormal, Point3D planePoint) {
        Point3D projection = projectToPlane(point, planeNormal, planePoint);
        return centralSymmetry(projection, point);
    }
    /**
     * 将三维点投影到平面上，并返回二维点
     * @param p 三维点
     * @param planeNormal 平面法向量（单位向量）
     * @param planePoint 平面上的一点
     * @return 投影后的二维点
     */
    public static Point2D toPoint2D(Point3D p, Point3D planeNormal, Point3D planePoint) {
        // 将点投影到平面上
        Point3D projected = projectToPlane(p, planeNormal, planePoint);
        // 返回投影后的点的x和y坐标
        return new Point2D(projected.x, projected.y);
    }
    /**
     * 给定三点计算平面法向量
     */
    public static Point3D calculatePlaneNormal(Point3D p1, Point3D p2, Point3D p3) {
        // 计算向量
        Point3D v1 = translate(p2, centralSymmetry(p1));
        Point3D v2 = translate(p3, centralSymmetry(p1));
        // 计算叉积得到法向量
        return normalize(cross(v1, v2));
    }
    /**
     * 计算从原点出发、经过点p的直线，与由(planeNormal, planePoint)定义的平面的交点
     *
     * 直线：X = t * p   (t ∈ ℝ, 起点为原点O)
     * 平面：n · (X - P0) = 0   (n = planeNormal, P0 = planePoint)
     *
     * 解方程可得：
     *   t = -(n · P0) / (n · p)
     *   交点 = t * p
     *
     * @param p 直线上的点（即方向向量，原点到p）
     * @param planeNormal 平面法向量（不需要归一化）
     * @param planePoint 平面上的任意一点
     * @return 平面上的交点；如果直线与平面平行或方向无效，则返回null
     */
    public static Point3D linePlaneIntersection(Point3D p, Point3D planeNormal, Point3D planePoint) {
        // 特殊情况：方向为零向量时，直线无效
        if (p.isZero()) return null;

        // 平面方程：n · X + d = 0，其中 d = -(n · P0)
        double d = -dot(planeNormal, planePoint);

        // 计算分母 n · p
        double denominator = dot(planeNormal, p);

        // 如果分母接近0，说明直线与平面平行 → 没有交点
        if (Math.abs(denominator) < 1e-9) {
            return null;
        }

        // 参数t = -(n·P0) / (n·p)
        double t = -d / denominator;

        // 交点 = t * p
        return scale(p, t);
    }


}

