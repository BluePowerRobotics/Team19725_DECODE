package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.Point3D;

/**
 * 用作为其他Class提供函数的Class
 */
public class SixServoArmController{
    private HardwareMap hardwareMap;//硬件地图
    private Telemetry telemetry;//遥测
    private SixServoArmOutputter Outputter;//舵机输出器
    private SixServoArmCalculator Calculator = new SixServoArmCalculator();//三角函数计算器
    private SixServoArmState State = SixServoArmState.getInstance();//机械臂状态存储器
    public SixServoArmOutputter getOutputter() {
        return Outputter;
    } //获取舵机输出器
    public SixServoArmCalculator getCalculator() {
        return Calculator;
    } //获取三角函数计算器
    public SixServoArmState getState() {
        return State;
    } //获取机械臂状态存储器
    private SixServoArmController(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry;
        this.Outputter = new SixServoArmOutputter(hardwareMap, telemetry);
    }//构造函数
    private static SixServoArmController instance; //实例，用以单例模式
    public static SixServoArmController getInstance(){
        return instance;
    }//获取单例，无论何时何地都是同一个
    public static void setInstance(HardwareMap hardwareMap, Telemetry telemetry){
        instance = new SixServoArmController(hardwareMap,telemetry);
    }//更新单例，使得每次启动OpMode都能够更新

    /**
     * 初始化机械臂
     *
     * @param TargetPoint 机械臂目标位置
     * @param ClipHeadingRadian 夹爪朝向与水平面间的弧度
     * @param RadianAroundArm3 夹爪绕机械臂第三节旋转的弧度
     */
    public void setTargetPosition(Point3D TargetPoint, double ClipHeadingRadian, double RadianAroundArm3){
        double[] result = Calculator.calculateToServo(TargetPoint,ClipHeadingRadian);
        Outputter.setRadian(result);
        Outputter.setRadian(4,RadianAroundArm3);
    }//设置机械臂目标位置

    /**
     * 设置夹爪锁定状态
     * @param ifLocked 是否锁定
     */
    public void setClip(boolean ifLocked){
        if(ifLocked){
            Outputter.setPosition(5,SixServoArmOutputter.ClipLockPosition);
        }else{
            Outputter.setPosition(5,SixServoArmOutputter.ClipOpenPosition);
        }
    }//设置夹爪锁定状态

    /**
     * 更新机械臂状态
     * @return 是否所有舵机都完成移动
     */
    public boolean update(){
        return State.ifFinishedMoving();
    }//更新机械臂状态，返回是否所有舵机都完成移动

    /**
     * 获取机械臂当前位置
     * @return 机械臂当前位置
     */
    public Point3D getCurrentPosition(){
        return Calculator.calculateFromServo(State.getServoNowDegree());
    }//获取机械臂当前位置
}
/**
 * 用作输出的Class
 */
@Config
class SixServoArmOutputter{
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    SixServoArmOutputter(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry;
        for(int i = 0; i < servo.length; i++){
            servo[i] = hardwareMap.get(Servo.class, "servoc" + i);
            if(i == 1 || i == 4){
                servo[i].setDirection(Servo.Direction.REVERSE);
            }else{
                servo[i].setDirection(Servo.Direction.FORWARD);
            }
        }//设置舵机及其方向
    }//构造函数

    public static double ClipOpenPosition = 0.3;
    public static double ClipLockPosition = 0.8;

    private Servo[] servo = new Servo[6];
    public static double[] servoZeroPositionDegree = {19.90211745904914,-44.9752747253,33.5013262599,48.535188216,-72.58297644539614,0};
    public static double[] servoRangeDegree = {179.7842588893328, 247.2527472527472, 265.25198938992037, 245.4991816693945, 240.89935760171304, 170};//舵机总旋转角度    public static double[] x1 ={0.09,0.24,0.87,0.47,0.32};
    public static double[] y1 ={0,0,180,180,0};
    public static double[] x2 ={0.37,0.58,0.52,0.13,0.69};
    public static double[] y2 ={90,90,90,90,90};

    /**
     * 将舵机数值转化为度数值
     * @param servoIndex 舵机索引
     * @param Position 舵机位置，范围为0-1
     * @return 舵机位置的度数值
     */
    static double toDegree(int servoIndex, double Position){
        return Position * servoRangeDegree[servoIndex] + servoZeroPositionDegree[servoIndex];
    }

    /**
     * 将度数值转化为舵机位置
     * @param servoIndex 舵机索引
     * @param Degree 舵机位置的度数值
     * @return 舵机位置，范围为0-1
     */
    static double toPosition(int servoIndex, double Degree){
        return ( Degree - servoZeroPositionDegree[servoIndex] ) / servoRangeDegree[servoIndex];
    }
    /**
     * 设置舵机位置
     * @param servoIndex 舵机索引
     * @param Position 舵机位置，范围为0-1
     */
    public void setPosition(int servoIndex, double Position) {
        if (servoIndex < 0 || servoIndex >= servo.length) {
            telemetry.addData("Error", "Servo index out of range: " + servoIndex);
            return;
        }
        if (Double.isNaN(Position)){
            telemetry.addData("Error", "Servo" + servoIndex + "position is NaN");
            return;
        }
        if (Position < 0 || Position > 1) {
            telemetry.addData("Error", "Servo"+servoIndex+"position out of range: " + Position);
            Position=Range.clip(Position, 0, 1); // 确保位置在0到1之间
        }
        SixServoArmState.getInstance().setServoTargetDegree(servoIndex, toDegree(servoIndex, Position));
        servo[servoIndex].setPosition(Position);
    }

    /**
     * 设置多个舵机位置
     * @param Positions 舵机位置数组，长度应与舵机数量相同或更短
     */
    public void setPosition (@NonNull double[] Positions) {
        if (Positions.length > servo.length) {
            telemetry.addData("Error", "Positions array length does not match servo count: " + Positions.length);
            return;
        }
        for (int i = 0; i < Positions.length; i++) {
            if (Double.isNaN(Positions[i])){
                telemetry.addData("Error", "Servo" + i + "position is NaN");
                continue;
            }
            if (Positions[i] < 0 || Positions[i] > 1) {
                telemetry.addData("Error", "Servo position out of range at index " + i + ": " + Positions[i]);
                Positions[i]=Range.clip(Positions[i], 0, 1); // 确保位置在0到1之间
            }
            SixServoArmState.getInstance().setServoTargetDegree(i, toDegree(i,Positions[i]));
            servo[i].setPosition(Positions[i]);
        }
    }
    /**
     * 设置舵机角度
     * @param servoIndex 舵机索引
     * @param Degree 舵机位置的度数值
     */
    public void setDegree(int servoIndex, double Degree) {
        if (servoIndex < 0 || servoIndex >= servo.length) {
            telemetry.addData("Error", "Servo index out of range: " + servoIndex);
            return;
        }
        if (Double.isNaN(Degree)){
            telemetry.addData("Error", "Servo" + servoIndex + "Degree is NaN");
            return;
        }
        if (Degree < servoZeroPositionDegree[servoIndex] || Degree > servoZeroPositionDegree[servoIndex] + servoRangeDegree[servoIndex]) {
            telemetry.addData("Error", "Servo degree out of range: " + Degree);
        }
        SixServoArmState.getInstance().setServoTargetDegree(servoIndex, Degree);
        double Position = toPosition(servoIndex, Degree);
        servo[servoIndex].setPosition(Position);
    }
    /**
     * 设置多个舵机角度
     * @param Degrees 舵机位置的度数值数组，长度应与舵机数量相同或更短
     */
    public void setDegree(@NonNull double[] Degrees) {
        if (Degrees.length > servo.length) {
            telemetry.addData("Error", "Degrees array length does not match servo count: " + Degrees.length);
            return;
        }
        for (int i = 0; i < Degrees.length; i++) {
            if (Degrees[i] < servoZeroPositionDegree[i] || Degrees[i] > servoZeroPositionDegree[i] + servoRangeDegree[i]) {
                telemetry.addData("Error", "Servo degree out of range at index " + i + ": " + Degrees[i]);
            }
            if (Double.isNaN(Degrees[i])){
                telemetry.addData("Error", "Servo" + i + "Degree is NaN");
                continue;
            }
            SixServoArmState.getInstance().setServoTargetDegree(i, Degrees[i]);
            double Position = toPosition(i, Degrees[i]);
            servo[i].setPosition(Position);
        }
    }
    /**
     * 设置舵机弧度
     * @param servoIndex 舵机索引
     * @param Radian 舵机位置的弧度值
     */
    public void setRadian(int servoIndex, double Radian) {
        if (servoIndex < 0 || servoIndex >= servo.length) {
            telemetry.addData("Error", "Servo index out of range: " + servoIndex);
            return;
        }
        double Degree = Math.toDegrees(Radian);
        setDegree(servoIndex, Degree);
    }
    /**
     * 设置多个舵机弧度
     * @param Radians 舵机位置的弧度值数组，长度应与舵机数量相同或更短
     */
    public void setRadian(@NonNull double[] Radians) {
        if (Radians.length > servo.length) {
            telemetry.addData("Error", "Radians array length does not match servo count: " + Radians.length);
            return;
        }
        for (int i = 0; i < Radians.length; i++) {
            double Degree = Math.toDegrees(Radians[i]);
            setDegree(i, Degree);
        }
    }
}
/**
 * 用作三角函数计算的Class
 */
@Config
class SixServoArmCalculator {

    SixServoArmCalculator() {
        //无需地图、遥测，纯计算
    }//构造函数
    static double endErrorToR=0;//末端偏离r轴距离（右正）
    static double Arm1ErrorToZ=0;//第一臂偏离z轴距离（前正）
    static double Arm1 = 153;
    static double Arm2 = 143;
    static double Arm3 = 125;
    private double[] result = {0,0,0,0};
    public double[] calculateToServo(@NonNull Point3D PointD, double RadianArm3ToHorizontal){
        Point2D PointDxy = new Point2D(PointD.getX(),PointD.getY());
        double RadianArm = PointDxy.getRadian();//计算水平面上与x轴的夹角
        double r = PointDxy.getDistance();
        //计算第二臂末端坐标
        Point2D PointDrz = new Point2D(r,PointD.getZ());
        Point2D PointC = Point2D.translateRD(PointDrz,Math.PI+RadianArm3ToHorizontal,Arm3);
        //解三角形（C，C在R轴投影，原点）
        double RadianC = PointC.getRadian();
        double PointCRadian = 0;
        PointCRadian += (0.5*Math.PI - RadianC)+RadianArm3ToHorizontal+0.5*Math.PI;
        //解三角形（A，B，C）
        double lengthAC = PointC.getDistance();
        //lengthAB = Arm1,lengthBC= Arm2
        double RadianA = Math.acos((Arm1 * Arm1 + lengthAC * lengthAC - Arm2 * Arm2) / (2 * Arm1 * lengthAC));
        double RadianB = Math.acos((Arm1 * Arm1 + Arm2 * Arm2 - lengthAC * lengthAC) / (2 * Arm1 * Arm2));
        PointCRadian += Math.PI - RadianA - RadianB;
        RadianA += RadianC;
        //返回结果
        result = new double[]{RadianArm,RadianA,RadianB,PointCRadian};
        return result;
    }
    Point3D calculateFromServo(@NonNull double[] Degree){
        Point3D AB = Point3D.fromSpherical(Math.toRadians(Degree[0]),Math.toRadians(90-Degree[1]),Arm1);
        double BCPolar = 270-Degree[2]-Degree[1];
        Point3D BC = Point3D.fromSpherical(Math.toRadians(Degree[0]),Math.toRadians(BCPolar),Arm2);
        Point3D CD = Point3D.fromSpherical(Math.toRadians(Degree[0]),Math.toRadians(180-Degree[3]+BCPolar),Arm3);
        Point3D PointA = Point3D.ZERO;
        Point3D PointB = Point3D.translate(PointA,AB);
        Point3D PointC = Point3D.translate(PointB,BC);
        Point3D PointD = Point3D.translate(PointC,CD);
        return PointD;
    }
}
/**
 *用作机械臂状态存储的Class
 */
 class SixServoArmState {
    private static SixServoArmState instance;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private SixServoArmState() {
        lastUpdateTime= System.currentTimeMillis();
    }//构造函数
    public static SixServoArmState getInstance() {
        if (instance == null) {
            instance = new SixServoArmState();
        }
        return instance;
    }//获取单例，无论何时何地都是同一个
    //状态存储相关函数
    private double[] servoNowDegree = {0,0,0,0,0,0}; //舵机当前位置数组
    public double[] getServoNowDegree() {
        update();
        return servoNowDegree;
    } //获取舵机当前位置数组
    private double[] servoTargetDegree = {0,0,0,0,0,0}; //舵机目标位置数组
    public double[] getServoTargetDegree() {
        return servoTargetDegree;
    } //获取舵机目标位置数组
    private double[] servoSpeed={0.36,0.24,0.24,0.24,0.24,0.24};//sec per 60 degree
    boolean[] servoFinishedMoving = {false, false, false, false, false, false}; //舵机是否完成正在移动
    boolean servoFinishedMovingAll = false; //所有舵机是否完成移动
    long lastUpdateTime = 0; //上次更新时间

    /**
     * 更新舵机状态
     * 对于外部使用，除非特殊情况，否则不应直接调用此函数
     * 特殊情况是指不会在很长一段时间内调用setServoTargetDegree和ifFinishedMoving函数
     */
    void update(){
        servoFinishedMovingAll= true; //假设所有舵机都完成移动
        long nowTime = System.currentTimeMillis();
        for (int i = 0; i < servoNowDegree.length; i++) {
            servoFinishedMoving[i] = false; //重置舵机是否完成移动标记
            if(servoTargetDegree[i]>servoNowDegree[i]){
                servoNowDegree[i] += 60 * (nowTime - lastUpdateTime) / 1000.0 / servoSpeed[i];
                if(servoNowDegree[i]>=servoTargetDegree[i]){
                    servoNowDegree[i]=servoTargetDegree[i];
                    servoFinishedMoving[i] = true; //标记为完成移动
                }
            }else if(servoTargetDegree[i]<servoNowDegree[i]) {
                servoNowDegree[i] -= 60 * (nowTime - lastUpdateTime) / 1000.0 / servoSpeed[i];
                if (servoNowDegree[i] <= servoTargetDegree[i]) {
                    servoNowDegree[i] = servoTargetDegree[i];
                    servoFinishedMoving[i] = true; //标记为完成移动
                }
            }else{
                servoFinishedMoving[i] = true; //如果目标位置和当前位置相同，标记为完成移动
            }
            if(!servoFinishedMoving[i]) {
                servoFinishedMovingAll = false; //如果有一个舵机没有完成移动，则标记为未完成
            }
        }
        lastUpdateTime = nowTime; //更新上次更新时间
    }
    void setServoTargetDegree(int servoIndex, double targetDegree) {
        servoTargetDegree[servoIndex] = targetDegree;
        update(); //更新舵机状态
    }
    boolean ifFinishedMoving() {
        update(); //更新舵机状态
        //检查所有舵机是否完成移动
        return servoFinishedMovingAll;
    }
}