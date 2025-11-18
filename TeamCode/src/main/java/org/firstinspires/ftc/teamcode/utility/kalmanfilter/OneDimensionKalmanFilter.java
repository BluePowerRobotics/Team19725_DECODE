package org.firstinspires.ftc.teamcode.utility.kalmanfilter;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.kalmanfilter.jama.*;

/// <summary>
/// 会根据随动轮移动的位置以更新估计位置 避免随动轮移动的积累误差
/// </summary>
@Config
public class OneDimensionKalmanFilter {
    long lastUpdateTime=System.nanoTime();
    double lastEsPosition=0.0;
    double lastEsVelocity=0.0;
    double lastWheelPosition=0.0;

    Matrix F=new Matrix(new double[][]{
            {1.0,1.0},
            {0.0,1.0}
        });
    /// 状态协方差矩阵
    Matrix P=new Matrix(new double[][]{
            {1.0,0.0},
            {0.0,1.0}
        });
    /// 观测矩阵 sb矩阵
    Matrix H=new Matrix(new double[][]{
            {1.0,0.0}
        });

    public static double q1 = 0.00001;

    //以下两个矩阵用于调整滤波器性能 决定观测更可信还是预测更可信
    /// 过程噪声协方差矩阵 
    Matrix Q=new Matrix(new double[][]{
            {q1,0.0},
            {0.0,q1}
        });
    /// 测量噪声协方差
    public static double R=1;

    public OneDimensionKalmanFilter(double initialPosition, double initialVelocity){
        lastEsPosition=initialPosition;
        lastEsVelocity=initialVelocity;
        lastWheelPosition=initialPosition;
    }
    /// <summary>
    /// 更新卡尔曼滤波结果
    /// </summary>
    /// <param name="wheelPosition">随动轮当前位置</param>
    /// <param name="measurementPosition">视觉位置，如果没有输入 则请输入Double.NaN</param>
    /// <returns>新的估计位置</returns>
    public PosVelTuple Update(double wheelPosition,double measurementPosition){
        long currentTime=System.nanoTime();
        double deltaTime=currentTime-lastUpdateTime;
        lastUpdateTime=currentTime;
        double dtSeconds=deltaTime/1000000.0;
        double deltaPosition=wheelPosition-lastWheelPosition;
        lastWheelPosition=wheelPosition;
        Matrix X_=new Matrix(new double[][]{
            {lastEsPosition+deltaPosition},
            {deltaPosition/dtSeconds}
        });
        if(Double.isNaN(measurementPosition)){
            lastEsVelocity=X_.get(1, 0);
            lastEsPosition=X_.get(0, 0);
            return new PosVelTuple(lastEsPosition, lastEsVelocity);
        }
        Matrix P_=F.times(P).times(F.transpose()).plus(Q); //2*2
        Matrix K=P_.times(H.transpose()).times(1.0/(H.times(P_).times(H.transpose()).get(0, 0)+R)); //1*2
        Matrix X=X_.plus(K.times(new Matrix(new double[][]{
            {measurementPosition}
        }).minus(H.times(X_))));
        P=(Matrix.identity(2, 2).minus(K.times(H))).times(P_);
        lastEsPosition=X.get(0, 0);
        lastEsVelocity=X.get(1, 0);
        return new PosVelTuple(lastEsPosition, lastEsVelocity);
    }
}
