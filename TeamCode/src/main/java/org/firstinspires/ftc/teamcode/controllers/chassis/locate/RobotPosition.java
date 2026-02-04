/**
 * 机器人位置类
 * 负责机器人的位置跟踪和更新
 */
package org.firstinspires.ftc.teamcode.controllers.chassis.locate;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;
@Config
public class RobotPosition {
    /**
     * 机器人位置参数类
     */
    public static class Params {
        int minUpdateIntervalMs = 1; // 最小更新时间间隔，单位毫秒

    }
    public static Params PARAMS = new Params(); // 参数实例
    
    /**
     * 设置最小更新时间间隔
     * @param interval 时间间隔（毫秒）
     */
    public void setMinUpdateIntervalMs(int interval){
        PARAMS.minUpdateIntervalMs=interval;
    }

    private static RobotPosition instance; // 单例实例
    
    /**
     * 获取RobotPosition实例
     * @return RobotPosition实例
     * @throws IllegalStateException 如果实例未初始化
     */
    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }
    
    /**
    * 初始化位置(全新)
    * @param hardwareMap 硬件映射
     * @param initialPosition 初始位置(inch)
     * @param initialHeadingRadian 初始朝向，弧度制
    * @return RobotPosition实例
     */
    public static RobotPosition refresh(HardwareMap hardwareMap,Point2D initialPosition,double initialHeadingRadian){
        instance=new RobotPosition();
        instance.initialPosition=initialPosition;
        instance.initialHeadingRadian=initialHeadingRadian;
        Data.instance.setPosition(initialPosition);
        Data.instance.headingRadian=initialHeadingRadian;
        instance.mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(-Data.instance.getPosition(DistanceUnit.INCH).getY(), +Data.instance.getPosition(DistanceUnit.INCH).getX(), Data.instance.headingRadian));
        return instance;
    }
    
    /**
     * 初始化位置(全新)
     * @param hardwareMap 硬件映射
     * @param pose2d 初始位置(roadrunner方向)
     * @return RobotPosition实例
     */
    public static RobotPosition refresh(HardwareMap hardwareMap, Pose2d pose2d) {
        return refresh(hardwareMap, MathSolver.toPoint2D(pose2d), pose2d.heading.log());
    }
    
    /**
     * 初始化位置(使用旧位置)
     * @param hardwareMap 硬件映射
     * @return RobotPosition实例
     */
    public static RobotPosition refresh(HardwareMap hardwareMap){
        instance=new RobotPosition();
        instance.initialPosition= Data.instance.getPosition(DistanceUnit.INCH);
        instance.initialHeadingRadian= Data.instance.headingRadian;
        instance.mecanumDrive = new MecanumDrive(hardwareMap,Data.getInstance().getPose2d());
        return instance;
    }

    /**
     * 私有构造函数，防止外部实例化
     */
    private RobotPosition(){
    }
    
    public MecanumDrive mecanumDrive; //  mecanum驱动实例
    public Point2D initialPosition=new Point2D(0,0); // 初始位置
    public double initialHeadingRadian=0; // 初始朝向，弧度制

    public long lastUpdateTime=0; // 上次更新时间

    /**
     * 更新机器人位置和速度
     */
    public void update(){
        if(System.currentTimeMillis()-lastUpdateTime<PARAMS.minUpdateIntervalMs){
            return;
        }
        // 更新位姿估计
        PoseVelocity2d poseVelocity2d = mecanumDrive.updatePoseEstimate();
        // 获取当前位姿
        Pose2d pose = mecanumDrive.localizer.getPose();
        // 更新角速度
        Data.instance.headingSpeedRadianPerSec=poseVelocity2d.angVel;

        // 更新线速度（转换坐标系）
        Data.instance.setSpeed(new Vector2d(-poseVelocity2d.linearVel.y,+poseVelocity2d.linearVel.x));

        // 更新位置（转换坐标系）
        Data.instance.setPosition(new Point2D(-pose.position.y,+pose.position.x));
        // 更新朝向
        Data.instance.headingRadian=pose.heading.log();//==toDouble()... what can I say, man?
        lastUpdateTime=System.currentTimeMillis();
    }
    
    /**
     * 获取机器人数据
     * @return Data实例，包含位置、速度和朝向信息
     */
    public Data getData(){
        update();
        return Data.instance;
    }
}
