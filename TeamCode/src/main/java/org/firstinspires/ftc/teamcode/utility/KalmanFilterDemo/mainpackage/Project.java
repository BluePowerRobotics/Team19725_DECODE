package org.firstinspires.ftc.teamcode.utility.KalmanFilterDemo.mainpackage;
import java.util.*;

public class Project {
    public static void main(String[] args) throws InterruptedException {
        OneDimKalmanFilter filter=new OneDimKalmanFilter(0.0, 0.0);
        //获取正态分布数组
        Random rand=new Random();
        double truePosition=0.0;
        double trueVelocity=10.0;
        long last=System.nanoTime();
        for(int i=0;i<2000;i++){
            long n=System.nanoTime();
            double dlt=n-last;
            last=n;
            truePosition+=trueVelocity*(dlt/1000000);
            double measuredPosition=truePosition+rand.nextGaussian()*1.0; //测量值，带有噪声
            PosVelTuple result=filter.Update(truePosition, measuredPosition);
            System.out.println(truePosition+" "+measuredPosition+" "+result.position);
        }
    }
}
