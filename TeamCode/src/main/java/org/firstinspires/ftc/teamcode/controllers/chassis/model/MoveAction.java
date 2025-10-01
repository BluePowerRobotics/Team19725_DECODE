package org.firstinspires.ftc.teamcode.controllers.chassis.model;

import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;

public class MoveAction {
    public Point2D targetPoint;
    Point2D startPoint;
    Point2D nowSpeed;
    public double targetRadian;
    double startRadian;
    long startTimeMS;
    double maxV;
    double maxA;
    double maxOmega;
    double arriveThresholdV;
    double arriveRadianThreshold;

    private MoveAction(Builder builder) {
        this.targetPoint = builder.targetPoint;
        this.startPoint = builder.startPoint;
        this.nowSpeed = builder.nowSpeed;
        this.targetRadian = builder.targetRadian;
        this.startRadian = builder.startRadian;
        this.startTimeMS = System.currentTimeMillis();
        this.maxV = builder.maxV;
        this.maxA = builder.maxA;
        this.maxOmega = builder.maxOmega;
        this.arriveThresholdV = builder.arriveThresholdV;
        this.arriveRadianThreshold = builder.arriveRadianThreshold;
        calculatePath();
    }
    @Override
    public String toString() {
        return "MoveAction{" +
                "\ntargetPoint=" + targetPoint +
                "\nstartPoint=" + startPoint +
                "\nnowSpeed=" + nowSpeed +
                "\ntargetRadian=" + targetRadian +
                "\nstartRadian=" + startRadian +
                "\nstartTimeMS=" + startTimeMS +
                "\nmaxV=" + maxV +
                "\nmaxA=" + maxA +
                "\nmaxOmega=" + maxOmega +
                "\narriveThresholdV=" + arriveThresholdV +
                "\narriveRadianThreshold=" + arriveRadianThreshold +
                "\nstartSpeed=" + startSpeed +
                "\nspeedUpEndTimeMS=" + speedUpEndTimeMS +
                "\nspeedDownStartTimeMS=" + speedDownStartTimeMS +
                "\narriveTimeMS=" + arriveTimeMS +
                "\nError=" + Error +
                "\nspeedUpDistance=" + speedUpDistance +
                "\nspeedDownDistance=" + speedDownDistance +
                "\ncruiseDistance=" + cruiseDistance +
                "\nhopeCurrentPoint=" + getHopeCurrentPoint().toString() +
                "\n}";
    }
    private void calculatePath(){
        //todo 计算路径(梯形速度曲线)
        //     1. 计算直线距离和角度差
        //     2. 计算加速的时间和距离
        //     3. 计算是否需要匀速的时间和距离
        //     4. 对称减速的时间和距离
        //     5. 计算总体分段
        Error=Point2D.translate(targetPoint,Point2D.centralSymmetry(startPoint));
        double speedAngleError = Math.abs(Error.Radian-nowSpeed.Radian);
        startSpeed = nowSpeed.Distance*Math.cos(speedAngleError);
        double speedError = maxV-startSpeed;
        double speedUpTimeS = speedError/maxA;
        speedUpDistance = (startSpeed+maxV)*speedUpTimeS/2;
        double speedDownTimeS = maxV/maxA;
        speedDownDistance = maxV*speedDownTimeS/2;

        if((speedDownDistance+speedUpDistance)>=Error.Distance){
            if(MathSolver.sgn(speedError)!=1){
                // 起始速度大于等于巡航速度，以最大加速度调整为巡航速度过缓，直接通过匀减速运动计算
                double allTimeS=2*Error.Distance/(startSpeed);
                speedUpEndTimeMS=(long)((1.0-(maxV/startSpeed))*allTimeS*1000)+System.currentTimeMillis();
                speedDownStartTimeMS=speedUpEndTimeMS;
                arriveTimeMS = System.currentTimeMillis()+(long)(allTimeS*1000);
            }else{
                // 起始速度小于巡航速度，计算可达到的最大速度
                double t1 = Math.sqrt((2*maxA*Error.Distance+startSpeed*startSpeed)/(2*maxA*maxA))-startSpeed/maxA;
                double t2 = Math.sqrt((2*maxA*Error.Distance+startSpeed*startSpeed)/(2*maxA*maxA));
                maxV=startSpeed+maxA*t1;
                speedUpEndTimeMS=(long)(t1*1000)+System.currentTimeMillis();
                speedDownStartTimeMS=speedUpEndTimeMS;
                arriveTimeMS = System.currentTimeMillis()+(long)((t1+t2)*1000);
            }
        }else{
            cruiseDistance = Error.Distance - speedUpDistance - speedDownDistance;
            double cruiseTimeS = cruiseDistance/maxV;
            speedUpEndTimeMS=(long)(speedUpTimeS*1000)+System.currentTimeMillis();
            speedDownStartTimeMS=(long)((speedUpTimeS+cruiseTimeS)*1000)+System.currentTimeMillis();
            arriveTimeMS = System.currentTimeMillis()+(long)((speedUpTimeS+cruiseTimeS+speedDownTimeS)*1000);
        }
    }
    double startSpeed;
    long speedUpEndTimeMS;
    long speedDownStartTimeMS;
    long arriveTimeMS;
    Point2D Error;

    public Point2D getHopeCurrentPoint(){
        long nowTimeMS = System.currentTimeMillis();
        if(nowTimeMS>=arriveTimeMS){
            hopeCurrentPoint=targetPoint;
            return hopeCurrentPoint;
        }

        if(nowTimeMS<=speedUpEndTimeMS){
            //加速阶段
            double distance = (startSpeed+maxV)*(nowTimeMS-startTimeMS)/2000;
            hopeCurrentPoint=Point2D.translate(startPoint,Point2D.fromPolar(Error.Radian,distance));
        }else if(nowTimeMS<=speedDownStartTimeMS){
            //匀速阶段
            double distance = speedUpDistance + maxV*(nowTimeMS-speedUpEndTimeMS)/1000;
            hopeCurrentPoint=Point2D.translate(startPoint,Point2D.fromPolar(Error.Radian,distance));
        }else{
            //减速阶段
            double distance = speedUpDistance + cruiseDistance + (maxV)*(arriveTimeMS-nowTimeMS)/2000;
            hopeCurrentPoint=Point2D.translate(startPoint,Point2D.fromPolar(Error.Radian,distance));
        }

        return hopeCurrentPoint;
    }
    public double getHopeCurrentHeadingRadian(){
        if(System.currentTimeMillis()-arriveTimeMS>=0){
            return targetRadian;
        }
        return (targetRadian-startRadian)*(System.currentTimeMillis()-startTimeMS)/(arriveTimeMS-startTimeMS)+startRadian;
    }
    double speedUpDistance;
    double speedDownDistance;
    double cruiseDistance=0;
    Point2D hopeCurrentPoint;

    public static class Builder {
        Point2D targetPoint = new Point2D(0, 0);
        Point2D startPoint = new Point2D(0, 0);
        Point2D nowSpeed = new Point2D(0,0);
        double targetRadian = 0;
        double startRadian = 0;
        double maxV = ChassisController.Params.maxV;
        double maxA= ChassisController.Params.maxA; // 最大加速度
        double maxOmega = ChassisController.Params.maxOmega;
        double arriveThresholdV = ChassisController.Params.zeroThresholdV;
        double arriveRadianThreshold = Math.toRadians(5);
        public Builder() {

        }

        public Builder setTargetPoint(Point2D targetPoint) {
            this.targetPoint = targetPoint;
            return this;
        }

        public Builder setTargetRadian(double targetRadian) {
            this.targetRadian = targetRadian;
            return this;
        }

        public Builder setNowSpeed(Point2D speed){
            this.nowSpeed=speed;
            return this;
        }

        public Builder setStartPoint(Point2D startPoint) {
            this.startPoint = startPoint;
            return this;
        }
        public Builder setStartRadian(double startRadian) {
            this.startRadian = startRadian;
            return this;
        }
        public Builder setMaxV(double maxV) {
            this.maxV = maxV;
            return this;
        }

        public Builder setMaxOmega(double maxOmega) {
            this.maxOmega = maxOmega;
            return this;
        }

        public Builder setArriveThresholdV(double arriveThresholdV) {
            this.arriveThresholdV = arriveThresholdV;
            return this;
        }

        public Builder setArriveRadianThreshold(double arriveRadianThreshold) {
            this.arriveRadianThreshold = arriveRadianThreshold;
            return this;
        }

        public MoveAction build() {
            return new MoveAction(this);
        }
    }
}