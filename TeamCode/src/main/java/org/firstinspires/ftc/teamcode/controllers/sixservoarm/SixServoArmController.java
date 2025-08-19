package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//用作为其他Class提供函数的Class
public class SixServoArmController{
    private HardwareMap hardwareMap;//硬件地图
    private Telemetry telemetry;//遥测

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
//用作输出的Class
class SixServoArmOutputter{
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private SixServoArmOutputter(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry;
    }//构造函数
}
//用作三角函数计算的Class
@Config
class SixServoArmCalculator {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private SixServoArmCalculator() {
        //无需地图、遥测，纯计算
    }//构造函数
    static double Arm1 = 153;
    static double Arm2 = 143;
    static double Arm3 = 125;
    private double[] result = {0,0,0,0};
    public double[] calculate(double x,double y,double z,double RadianArm3ToHorizontal){
        double theta = 0;
        theta = Math.atan2(y,x);
        double r = Math.sqrt(x*x+y*y);
        //计算第二臂末端坐标
        Point2D PointD = new Point2D(r,z);
        Point2D PointC = Point2D.translateRD(PointD,RadianArm3ToHorizontal,Arm3);
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
        result = new double[]{theta,RadianA,RadianB,PointCRadian};
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
}
//用作机械臂状态存储的Class
class SixServoArmState {
    private static SixServoArmState instance;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private SixServoArmState() {
    }//构造函数
    public static SixServoArmState getInstance() {
        if (instance == null) {
            instance = new SixServoArmState();
        }
        return instance;
    }//获取单例，无论何时何地都是同一个
    //状态存储相关函数
}