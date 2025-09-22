package org.firstinspires.ftc.teamcode.controllers.chassis.model;

import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.utility.Point2D;

public class MoveAction {
    Point2D targetPoint;
    Point2D startPoint;
    double targetRadian;
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
        this.targetRadian = builder.targetRadian;
        this.startRadian = builder.startRadian;
        this.startTimeMS = System.currentTimeMillis();
        this.maxV = builder.maxV;
        this.maxA = builder.maxA;
        this.maxOmega = builder.maxOmega;
        this.arriveThresholdV = builder.arriveThresholdV;
        this.arriveRadianThreshold = builder.arriveRadianThreshold;
    }
    private void calculatePath(){
        //todo 计算路径(梯形速度曲线)
        //     1. 计算直线距离和角度差
        //     2. 计算加速的时间和距离
        //     3. 计算是否需要匀速的时间和距离
        //     4. 对称减速的时间和距离
        //     5. 计算总体分段
    }
    long speedUpEndTimeMS;
    long speedDownStartTimeMS;
    long arriveTimeMS;
    Point2D Error;
    Point2D hopeCurrentPoint;

    public static class Builder {
        Point2D targetPoint = new Point2D(0, 0);
        Point2D startPoint = new Point2D(0, 0);
        double targetRadian = 0;
        double startRadian = 0;
        double maxV = ChassisController.Params.maxV;
        double maxA= ChassisController.Params.maxA; // 最大加速度
        double maxOmega = ChassisController.Params.maxOmega;
        double arriveThresholdV = ChassisController.Params.zeroThresholdV;
        double arriveRadianThreshold = Math.toRadians(5);

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