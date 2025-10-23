//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.ejml.simple.SimpleMatrix;   // FTC SDK 自带 EJML
//import org.firstinspires.ftc.teamcode.utility.RobotEKF;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name="EKF_Demo")
//public class EKFDemo extends LinearOpMode {
//    // C++ 对象通过 JNI 或简单保留字段都可以；这里给出纯 Java 移植版
//    RobotEKF ekf = new RobotEKF();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        waitForStart();
//
//        while (opModeIsActive()) {
//            /* 1. 底盘预测 -------------------------- */
//            double vX     = -gamepad1.left_stick_y * 1.0;  // m/s
//            double omega  =  gamepad1.right_stick_x * 3.0; // rad/s
//            double dt     = 0.030;                         // 30 ms 主循环
//            ekf.predict(vX, omega, dt);
//
//            /* 2. 视觉更新（Limelight/AprilTag）----- */
//            if (limelight.hasNewBotPose()) {
//                double[] bot = limelight.getBotpose();   // [x,y,θ]
//                SimpleMatrix z = new SimpleMatrix(3,1,true,bot);
//                // 根据 tag 数量动态调噪声
//                double n = limelight.getTagCount();
//                double std = 0.30 / Math.sqrt(n);
//                SimpleMatrix R = new SimpleMatrix(3,1,true,
//                        std, std, Math.toRadians(5)/Math.sqrt(n));
//                ekf.update(z, R);
//            }
//
//            /* 3. 实时telemetry --------------------- */
//            telemetry.addData("EKF x" , "%.2f m", ekf.x());
//            telemetry.addData("EKF y" , "%.2f m", ekf.y());
//            telemetry.addData("EKF θ°", "%.1f°", Math.toDegrees(ekf.theta()));
//            telemetry.update();
//        }
//    }
//}