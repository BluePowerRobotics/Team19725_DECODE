package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * 用作为其他Class提供函数的Class
 */
public class SixServoArmController{
    private HardwareMap hardwareMap;//硬件地图
    private Telemetry telemetry;//遥测
    private SixServoArmOutputter Outputter = new SixServoArmOutputter(hardwareMap, telemetry);//舵机输出器
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
    }//构造函数
    private static SixServoArmController instance; //实例，用以单例模式
    public static SixServoArmController getInstance(){
        return instance;
    }//获取单例，无论何时何地都是同一个
    public static void setInstance(HardwareMap hardwareMap, Telemetry telemetry){
        instance = new SixServoArmController(hardwareMap,telemetry);
    }//更新单例，使得每次启动OpMode都能够更新

}
/**
 * 用作输出的Class
 */
class SixServoArmOutputter{
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    SixServoArmOutputter(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry;
        for(int i = 0; i < servo.length; i++){
            servo[i] = hardwareMap.get(Servo.class, "servoe" + i);
            if(i == 2 || i == 3 || i == 4){
                servo[i].setDirection(Servo.Direction.REVERSE);
            }else{
                servo[i].setDirection(Servo.Direction.FORWARD);
            }
        }//设置舵机及其方向
    }//构造函数

    private Servo[] servo = new Servo[6];
    public static double[] servoZeroPositionDegree = {-28.92857,-63.529411,-43.71428,55.5882 , -77.83783783783785, 0};
    public static double[] servoRangeDegree = {321.42857, 264.70588, 257.142847142857142857, 264.70588, 243.24324324324328, 170};//舵机总旋转角度
    public static double[] x1 ={0.09,0.24,0.87,0.47,0.32};
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
        if (Position < 0 || Position > 1) {
            telemetry.addData("Error", "Servo position out of range: " + Position);
            Position=Range.clip(Position, 0, 1); // 确保位置在0到1之间
        }
        SixServoArmState.getInstance().setServoTargetDegree(servoIndex, toDegree(servoIndex, Position));
        servo[servoIndex].setPosition(Position);
    }

    /**
     * 设置多个舵机位置
     * @param Positions 舵机位置数组，长度应与舵机数量相同或更短
     */
    public void setPosition (double[] Positions) {
        if (Positions.length > servo.length) {
            telemetry.addData("Error", "Positions array length does not match servo count: " + Positions.length);
            return;
        }
        for (int i = 0; i < Positions.length; i++) {
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
        if (Degree < servoZeroPositionDegree[servoIndex] || Degree > servoZeroPositionDegree[servoIndex] + servoRangeDegree[servoIndex]) {
            telemetry.addData("Error", "Servo degree out of range: " + Degree);
            return;
        }
        SixServoArmState.getInstance().setServoTargetDegree(servoIndex, Degree);
        double Position = toPosition(servoIndex, Degree);
        servo[servoIndex].setPosition(Position);
    }
    /**
     * 设置多个舵机角度
     * @param Degrees 舵机位置的度数值数组，长度应与舵机数量相同或更短
     */
    public void setDegree(double[] Degrees) {
        if (Degrees.length > servo.length) {
            telemetry.addData("Error", "Degrees array length does not match servo count: " + Degrees.length);
            return;
        }
        for (int i = 0; i < Degrees.length; i++) {
            if (Degrees[i] < servoZeroPositionDegree[i] || Degrees[i] > servoZeroPositionDegree[i] + servoRangeDegree[i]) {
                telemetry.addData("Error", "Servo degree out of range at index " + i + ": " + Degrees[i]);
                continue; // 跳过不合法的角度
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
    public void setRadian(double[] Radians) {
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
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    SixServoArmCalculator() {
        //无需地图、遥测，纯计算
    }//构造函数
    static double Arm1 = 153;
    static double Arm2 = 143;
    static double Arm3 = 125;
    private double[] result = {0,0,0,0};
    public double[] calculateToServo(double x, double y, double z, double RadianArm3ToHorizontal){
        Point2D PointDxy = new Point2D(x,y);
        double RadianArm = PointDxy.Radian;//计算水平面上与x轴的夹角
        double r = PointDxy.Distance;
        //计算第二臂末端坐标
        Point2D PointDrz = new Point2D(r,z);
        Point2D PointC = Point2D.translateRD(PointDrz,RadianArm3ToHorizontal,Arm3);
        //解三角形（C，C在R轴投影，原点）
        double RadianC = Math.atan2(PointC.y, PointC.x);
        double PointCRadian = 0;
        PointCRadian += (0.5*Math.PI - RadianC)+RadianArm3ToHorizontal+0.5*Math.PI;
        //解三角形（A，B，C）
        double lengthAC = Math.sqrt(PointC.x * PointC.x + PointC.y * PointC.y);
        //lengthAB = Arm1,lengthBC= Arm2
        double RadianA = Math.acos((Arm1 * Arm1 + lengthAC * lengthAC - Arm2 * Arm2) / (2 * Arm1 * lengthAC));
        double RadianB = Math.acos((Arm1 * Arm1 + Arm2 * Arm2 - lengthAC * lengthAC) / (2 * Arm1 * Arm2));
        PointCRadian += Math.PI - RadianA - RadianB;
        RadianA += RadianC;
        //返回结果
        result = new double[]{RadianArm,RadianA,RadianB,PointCRadian};
        return result;
    }
    public double getRadian0() {
        return result[0];
    }
    public double getRadian1() {
        return result[1];
    }
    public double getRadian2() {
        return result[2];
    }
    public double getRadian3() {
        return result[3];
    }
    Point3D calculateFromServo(){
        return new Point3D(0,0,0);
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
    double[] servoNowDegree = {0,0,0,0,0}; //舵机当前位置数组
    double[] servoTargetDegree = {0,0,0,0,0}; //舵机目标位置数组
    private double[] servoSpeed={0.36,0.24,0.24,0.24,0.24};//sec per 60 degree
    boolean[] servoFinishedMoving = {false, false, false, false, false}; //舵机是否完成正在移动
    boolean servoFinishedMovingAll = false; //所有舵机是否完成移动
    long lastUpdateTime = 0; //上次更新时间
    void update(){
        servoFinishedMovingAll= true; //假设所有舵机都完成移动
        for (int i = 0; i < servoNowDegree.length; i++) {
            servoFinishedMoving[i] = false; //重置舵机是否完成移动标记
            if(servoTargetDegree[i]>servoNowDegree[i]){
                servoNowDegree[i] += 60 * (System.currentTimeMillis() - lastUpdateTime) / 1000.0 / servoSpeed[i];
                if(servoNowDegree[i]>=servoTargetDegree[i]){
                    servoNowDegree[i]=servoTargetDegree[i];
                    servoFinishedMoving[i] = true; //标记为完成移动
                }
            }else if(servoTargetDegree[i]<servoNowDegree[i]) {
                servoNowDegree[i] -= 60 * (System.currentTimeMillis() - lastUpdateTime) / 1000.0 / servoSpeed[i];
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
        lastUpdateTime = System.currentTimeMillis(); //更新上次更新时间
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