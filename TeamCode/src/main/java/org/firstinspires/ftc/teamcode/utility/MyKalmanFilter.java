package org.firstinspires.ftc.teamcode.utility;

import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.ArrayRealVector;

public class MyKalmanFilter {
    private KalmanFilter filter;

    public MyKalmanFilter() {
        // 状态转移矩阵
        RealMatrix A = new Array2DRowRealMatrix(new double[][] { { 1 } });
        // 控制矩阵
        RealMatrix B = null;
        // 观测矩阵
        RealMatrix H = new Array2DRowRealMatrix(new double[][] { { 1 } });
        // 过程噪声协方差
        RealMatrix Q = new Array2DRowRealMatrix(new double[][] { { 1e-5 } });
        // 测量噪声协方差
        RealMatrix R = new Array2DRowRealMatrix(new double[][] { { 1e-2 } });
        // 初始状态估计
        RealVector initialStateEstimate = new ArrayRealVector(new double[] { 0 });
        // 初始误差协方差
        RealMatrix initialErrorCovariance = new Array2DRowRealMatrix(new double[][] { { 1 } });

        DefaultProcessModel pm = new DefaultProcessModel(A, B, Q, initialStateEstimate, initialErrorCovariance);
        DefaultMeasurementModel mm = new DefaultMeasurementModel(H, R);

        filter = new KalmanFilter(pm, mm);
    }

    public double update(double measurement) {
        filter.predict();
        filter.correct(new double[] { measurement });
        return filter.getStateEstimationVector().toArray()[0];
    }
}