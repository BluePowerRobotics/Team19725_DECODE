package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * 绘图工具类，用于在Dashboard画布上绘制机器人和相关元素。
 * 
 * 此类提供了静态方法来可视化机器人的位置、姿态和轨迹等信息，
 * 帮助调试和监控机器人的运动状态。
 */
public final class Drawing {
    /**
     * 私有构造函数，防止实例化此类。
     * 因为此类只包含静态方法，不需要创建实例。
     */
    private Drawing() {}

    /**
     * 在画布上绘制机器人。
     * 
     * 此方法绘制一个圆形表示机器人主体，并绘制一条线表示机器人的朝向。
     * 
     * @param c 画布对象，用于绘制机器人
     * @param t 机器人的姿态（位置和朝向）
     */
    public static void drawRobot(Canvas c, Pose2d t) {
        // 机器人半径（单位：英寸）
        final double ROBOT_RADIUS = 9;

        // 设置线条宽度为1
        c.setStrokeWidth(1);
        // 绘制机器人主体（圆形）
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        // 计算表示朝向的线段
        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        // 绘制朝向线段
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
