package org.firstinspires.ftc.teamcode.controllers.locate;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utility.Point2D;

public class RobotPosition {
    public static class Params {
        //todo 在MecanumDrive中调整参数
        static double inPerTick = MecanumDrive.PARAMS.inPerTick;
        static int minUpdateIntervalMs = 20; // 最小更新时间间隔，单位毫秒
        static DistanceUnit distanceUnit = DistanceUnit.MM;
        static UnnormalizedAngleUnit angleUnit = UnnormalizedAngleUnit.RADIANS;

    }
    public void setMinUpdateIntervalMs(@NonNull int interval){
        Params.minUpdateIntervalMs=interval;
    }

    private static RobotPosition instance;
    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }
    /**
    * 初始化位置(全新)
    * @param hardwareMap 硬件映射
     * @param initialPosition 初始位置
     * @param initialHeadingRadian 初始朝向，弧度制
    * @return RobotPosition实例
     */
    public static RobotPosition refresh(HardwareMap hardwareMap,Point2D initialPosition,double initialHeadingRadian){
        instance=new RobotPosition(hardwareMap);
        instance.initialPosition=initialPosition;
        instance.initialHeadingRadian=initialHeadingRadian;
        instance.pinpointLocalizer=new PinpointLocalizer(hardwareMap, Params.inPerTick, new Pose2d(instance.initialPosition.x,instance.initialPosition.y,instance.initialHeadingRadian));
        instance.pinpointLocalizer.setUnits(DistanceUnit.MM, UnnormalizedAngleUnit.RADIANS);
        Data.instance.position=initialPosition;
        Data.instance.headingRadian=initialHeadingRadian;
        return instance;
    }
    /**
     * 初始化位置(旧位置)
     * @param hardwareMap 硬件映射
     * @return RobotPosition实例
     */
    public static RobotPosition refresh(HardwareMap hardwareMap){
        instance=new RobotPosition(hardwareMap);
        instance.initialPosition= Data.instance.position;
        instance.initialHeadingRadian= Data.instance.headingRadian;
        instance.pinpointLocalizer=new PinpointLocalizer(hardwareMap, Params.inPerTick, new Pose2d(instance.initialPosition.x,instance.initialPosition.y,instance.initialHeadingRadian));
        instance.pinpointLocalizer.setUnits(DistanceUnit.MM, UnnormalizedAngleUnit.RADIANS);
        return instance;
    }

    private RobotPosition(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
        }

    private HardwareMap hardwareMap;
    private PinpointLocalizer pinpointLocalizer;
    public Point2D initialPosition=new Point2D(0,0);
    public double initialHeadingRadian=0;

    public long lastUpdateTime=0;

    public void update(){
        if(System.currentTimeMillis()-lastUpdateTime<Params.minUpdateIntervalMs){
            return;
        }
        Data.instance.headingSpeedRadianPerSec=pinpointLocalizer.update().angVel;
        Data.instance.speed=pinpointLocalizer.getWorldVelocity();
        Pose2d pose=pinpointLocalizer.getPose();
        Data.instance.position=new Point2D(pose.position.x,pose.position.y);
        Data.instance.headingRadian=pose.heading.log();//==toDouble()... what can I say, man?
        lastUpdateTime=System.currentTimeMillis();
    }
    public Data getData(){
        update();
        return Data.instance;
    }
}
