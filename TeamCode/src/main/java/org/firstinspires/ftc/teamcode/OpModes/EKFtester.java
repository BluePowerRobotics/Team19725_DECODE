package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.OneDimensionKalmanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.PosVelTuple;

@TeleOp
public class EKFtester extends LinearOpMode {
    Pose2d[] vis = {new Pose2d(0,0,0),
            new Pose2d(Double.NaN,Double.NaN,  Double.NaN)
    };
    Pose2d[] odo = {};
    Pose2d[] ans = {};

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OneDimensionKalmanFilter kalman_x = new OneDimensionKalmanFilter(0,0);
        OneDimensionKalmanFilter kalman_y = new OneDimensionKalmanFilter(0,0);
        //OneDimensionKalmanFilter kalman_theta = new OneDimensionKalmanFilter(0,0);
        waitForStart();
        int i = 0;
        while(opModeIsActive() && i < vis.length){
            double distanceX = Math.min(vis[i].position.x - 72, vis[i].position.x + 72);
            double distanceY = vis[i].position.y + 72;
            double distance = Math.sqrt(distanceY * distanceY+ distanceX * distanceX);
            PosVelTuple result_x=kalman_x.Update(odo[i].position.x, vis[i].position.x, distance);
            PosVelTuple result_y=kalman_y.Update(odo[i].position.y, vis[i].position.y, distance);
            Pose2d tmp = new Pose2d(result_x.position,result_y.position,odo[i].heading.toDouble());
            ans[i] = tmp;
            i++;
            telemetry.addData("odo_x", odo[i].position.x);
            telemetry.addData("odo_y", odo[i].position.y);
            if(!Double.isNaN(vis[i].position.x)){
                telemetry.addData("vis_x", vis[i].position.x);
                telemetry.addData("vis_y", vis[i].position.y);
            }
            telemetry.addData("ans_x", ans[i].position.x);
            telemetry.addData("ans_y", ans[i].position.y);
            telemetry.update();
        }
    }
}
