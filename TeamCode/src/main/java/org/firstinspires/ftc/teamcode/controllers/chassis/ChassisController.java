package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.chassis.locate.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
public class ChassisController {

    public static class Params{
        //todo 调整参数
        public double maxV=0.5; // 最大线速度 (m/s)
        public double maxA=0.5; // 最大加速度 (m/s²)
        public double maxOmega=Math.PI*1/2; // 最大角速度 (rad/s)
        public double zeroThresholdV =0.05; // 速度零点阈值 (m/s)
        public double zeroThresholdOmega =Math.toRadians(5); // 角速度零点阈值 (rad/s)
    }
    public static Params PARAMS = new Params();
    public RobotPosition robotPosition;
    HardwareMap hardwareMap;
    boolean fullyAutoMode = false;
    boolean useNoHeadMode=false;
    public boolean runningToPoint=false;
    boolean autoLockHeading=true;
    boolean HeadingLockRadianReset=true;
    double HeadingLockRadian;
    public double noHeadModeStartError;
    public ActionRunner actionRunner=new ActionRunner();
    public static org.firstinspires.ftc.teamcode.controllers.chassis.ChassisCalculator chassisCalculator= new ChassisCalculator();
    ChassisOutputter chassisOutputter;

    /**
     * OpMode初始化时调用
     */
    public ChassisController(HardwareMap hardwareMap){
        robotPosition= RobotPosition.refresh(hardwareMap);
        this.hardwareMap=hardwareMap;
        chassisOutputter=new ChassisOutputter(hardwareMap);
        HeadingLockRadian = robotPosition.getData().headingRadian;
        noHeadModeStartError=robotPosition.getData().headingRadian;
    }

    /**
     * Auto初始化时调用
     * @param hardwareMap 硬件映射
     * @param initialPosition 初始位置
     * @param initialHeadingRadian 初始朝向，弧度制
     */
    public ChassisController(HardwareMap hardwareMap, Point2D initialPosition, double initialHeadingRadian){
        robotPosition= RobotPosition.refresh(hardwareMap,initialPosition,initialHeadingRadian);
        this.hardwareMap=hardwareMap;
        //fullyAutoMode=true;
        chassisOutputter=new ChassisOutputter(hardwareMap);
        HeadingLockRadian = robotPosition.getData().headingRadian;
        noHeadModeStartError=robotPosition.getData().headingRadian;
    }
    /**
     * Auto初始化时调用
     * @param hardwareMap 硬件映射
     * @param initialPose 初始位置（roadrunner方向）
     */
    public ChassisController(HardwareMap hardwareMap, Pose2d initialPose){
        this(hardwareMap,new Point2D(+initialPose.position.y,-initialPose.position.x),initialPose.heading.log());
    }
    public void exchangeNoHeadMode(){
        useNoHeadMode=!useNoHeadMode;
    }
    public boolean getUseNoHeadMode(){
        return useNoHeadMode;
    }
    public void setAutoLockHeading(boolean autoLockHeading){
        this.autoLockHeading=autoLockHeading;
    }

    public void resetNoHeadModeStartError(double Radian){
        noHeadModeStartError = Radian;
    }
    public void resetNoHeadModeStartError(){
        resetNoHeadModeStartError(0);
    }
    public void setHeadingLockRadian(double headingLockRadian){
        this.HeadingLockRadian = MathSolver.normalizeAngle(headingLockRadian);
    }

    public void setTargetPoint(Pose2d pose2d){
        runningToPoint = true;
        TrajectoryActionBuilder actionBuilder = robotPosition.mecanumDrive.actionBuilder(robotPosition.getData().getPose2d())
                .strafeToLinearHeading(pose2d.position,pose2d.heading);
        actionRunner.clear();
        actionRunner.add(actionBuilder.build());
        HeadingLockRadian = pose2d.heading.log();
    }

    public void resetPosition(Pose2d pose2d){
        robotPosition=RobotPosition.refresh(hardwareMap,pose2d);
    }

    public double[] wheelSpeeds={0,0,0,0};
    public void gamepadInput(double vx,double vy,double omega){
        vx=vx*PARAMS.maxV;
        vy=vy*PARAMS.maxV;
        omega=omega*PARAMS.maxOmega;
        if(!fullyAutoMode){
            if(runningToPoint){
                if (Math.abs(Math.hypot(vx,vy))>=PARAMS.zeroThresholdV||Math.abs(omega)>=PARAMS.zeroThresholdOmega) {
                    runningToPoint = false;//打断自动驾驶
                }else{
                    actionRunner.update();
                    if(!actionRunner.isBusy()){
                        runningToPoint = false;
                    }
                }
            }
            if(!runningToPoint) {
                if(autoLockHeading){
                    if(omega!=0){
                        HeadingLockRadianReset=true;
                    }else{
                        if(HeadingLockRadianReset){
                            HeadingLockRadianReset=false;
                            HeadingLockRadian=robotPosition.getData().headingRadian;
                        }
                        omega=chassisCalculator.calculatePIDRadian(HeadingLockRadian,robotPosition.getData().headingRadian);
                    }
                }
                if (useNoHeadMode)
                    wheelSpeeds = chassisCalculator.solveGround(vx, vy, omega, robotPosition.getData().headingRadian-noHeadModeStartError);
                else
                    wheelSpeeds = chassisCalculator.solveChassis(vx, vy, omega);
                chassisOutputter.setWheelVelocities(wheelSpeeds);
            }
        }
    }
}

