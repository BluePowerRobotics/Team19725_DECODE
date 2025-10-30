package org.firstinspires.ftc.teamcode.utility.fakehardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.InstanceTelemetry;

public class FakeHardwareTest extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.setInstance(telemetry);
        DcMotor dcMotor = new FakeDcMotor();
        Servo servo = new FakeServo();
        waitForStart();
        while (opModeIsActive()) {
            dcMotor.setPower(-gamepad1.left_stick_y);
            servo.setPosition(-gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
