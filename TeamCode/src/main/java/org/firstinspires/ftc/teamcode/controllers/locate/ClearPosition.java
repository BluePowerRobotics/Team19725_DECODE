package org.firstinspires.ftc.teamcode.controllers.locate;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Point2D;

@TeleOp(name = "ClearPosition", group = "TEST")
public class ClearPosition extends LinearOpMode {
    /**
     * Clear the robot's position data
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Data.instance.setPosition(new Point2D(0, 0));
        Data.instance.headingRadian = 0;
        Data.instance.setSpeed(new Vector2d(0, 0));
        Data.instance.headingSpeedRadianPerSec = 0;
        telemetry.addData("ClearPosition", "Position data cleared");
        telemetry.update();
    }
}
