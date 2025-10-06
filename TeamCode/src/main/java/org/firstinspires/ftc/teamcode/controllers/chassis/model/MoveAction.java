package org.firstinspires.ftc.teamcode.controllers.chassis.model;

import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.utility.Point2D;

public class MoveAction {
    public Point2D targetPoint;
    Point2D startPoint;
    public double targetRadian;
    double startRadian;
    long startTimeMS;
    double vel;
    double arriveThresholdV;
    double arriveRadianThreshold;

    private MoveAction(Builder builder) {
        this.targetPoint = builder.targetPoint;
        this.startPoint = builder.startPoint;
        this.targetRadian = builder.targetRadian;
        this.startRadian = builder.startRadian;
        this.startTimeMS = System.currentTimeMillis();
        this.vel = builder.vel;
        this.arriveThresholdV = builder.arriveThresholdV;
        this.arriveRadianThreshold = builder.arriveRadianThreshold;
        calculatePath();
    }
    @Override
    public String toString() {
        return "MoveAction{" +
                "\ntargetPoint=" + targetPoint +
                "\nstartPoint=" + startPoint +
                "\ntargetRadian=" + targetRadian +
                "\nstartRadian=" + startRadian +
                "\nstartTimeMS=" + startTimeMS +
                "\nVel=" + vel +
                "\narriveThresholdV=" + arriveThresholdV +
                "\narriveRadianThreshold=" + arriveRadianThreshold +
                "\nError=" + Error +
                "\nhopeCurrentPoint=" + getHopeCurrentPoint().toString() +
                "\nhopeCurrentRadian=" + getHopeCurrentHeadingRadian() +
                "\n}";
    }
    private void calculatePath(){
        //自闭了，准备换成简单的直线写法
        Error=Point2D.translate(targetPoint,Point2D.centralSymmetry(startPoint));
        spendMS = (long)(Error.Distance/vel*1000);
    }
    long startMS=0;
    long spendMS;
    Point2D Error;

    public Point2D getHopeCurrentPoint(){
        long nowTimeMS = System.currentTimeMillis();
        if(startMS==0){
            startMS=nowTimeMS;
        }
        if(nowTimeMS-startMS>=spendMS){
            hopeCurrentPoint=targetPoint;
        }else {
            hopeCurrentPoint = new Point2D(
                    startPoint.x + Error.x * (nowTimeMS - startMS) / spendMS,
                    startPoint.y + Error.y * (nowTimeMS - startMS) / spendMS
            );
        }
        return hopeCurrentPoint;
    }
    public double getHopeCurrentHeadingRadian(){
        long nowTimeMS = System.currentTimeMillis();
        if(startMS==0){
            startMS=nowTimeMS;
        }
        if(nowTimeMS-(startMS+spendMS)>=0){
            return targetRadian;
        }
        return (targetRadian-startRadian)*(nowTimeMS-startTimeMS)/(spendMS)+startRadian;
    }
    Point2D hopeCurrentPoint;

    public static class Builder {
        Point2D targetPoint = new Point2D(0, 0);
        Point2D startPoint = new Point2D(0, 0);
        double targetRadian = 0;
        double startRadian = 0;
        double vel = ChassisController.Params.maxV;
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

        public Builder setStartPoint(Point2D startPoint) {
            this.startPoint = startPoint;
            return this;
        }
        public Builder setStartRadian(double startRadian) {
            this.startRadian = startRadian;
            return this;
        }
        public Builder setVel(double vel) {
            this.vel = vel;
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