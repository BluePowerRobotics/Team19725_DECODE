package org.firstinspires.ftc.teamcode.controllers.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@Config
public class TurretAngleController {
    private Servo angleServo;
    private DcMotorEx angleMotor;
    private boolean useServo;
    
    private PIDController anglePID;
    private double currentAngle;
    private double targetAngle;
    
    public static double servoMinPosition = 0.0;
    public static double servoMaxPosition = 1.0;
    public static double servo45Position = 0.5;
    
    public static double angleP = 0.1;
    public static double angleI = 0.01;
    public static double angleD = 0.05;
    public static double maxAngleI = 0.5;
    
    public static double angleTolerance = 0.05;
    
    private Telemetry telemetry;
    
    public TurretAngleController(HardwareMap hardwareMap, Telemetry telemetry, String servoName, boolean useServo) {
        this.telemetry = telemetry;
        this.useServo = useServo;
        
        if (useServo) {
            angleServo = hardwareMap.get(Servo.class, servoName);
            angleServo.setDirection(Servo.Direction.FORWARD);
            currentAngle = servo45Position;
            targetAngle = Math.PI / 4;
        } else {
            angleMotor = hardwareMap.get(DcMotorEx.class, servoName);
            angleMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            angleMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            angleMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            angleMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            
            anglePID = new PIDController(angleP, angleI, angleD, maxAngleI);
            currentAngle = 0;
            targetAngle = Math.PI / 4;
        }
    }
    
    public void setAngle(double angleRadians) {
        targetAngle = angleRadians;
        
        if (useServo) {
            double servoPosition = mapAngleToServoPosition(angleRadians);
            angleServo.setPosition(servoPosition);
            currentAngle = angleRadians;
        }
    }
    
    public void setAngleDegrees(double angleDegrees) {
        setAngle(Math.toRadians(angleDegrees));
    }
    
    public void setTo45Degrees() {
        setAngleDegrees(45);
    }
    
    public void update() {
        if (!useServo) {
            double error = targetAngle - currentAngle;
            double output = anglePID.calculate(error, 0, 0.02);
            
            double power = Math.max(-1, Math.min(1, output));
            angleMotor.setPower(power);
            
            currentAngle += power * 0.02;
        }
    }
    
    public boolean isAtTarget() {
        return Math.abs(targetAngle - currentAngle) < angleTolerance;
    }
    
    public double getCurrentAngle() {
        return currentAngle;
    }
    
    public double getCurrentAngleDegrees() {
        return Math.toDegrees(currentAngle);
    }
    
    public double getTargetAngle() {
        return targetAngle;
    }
    
    public double getTargetAngleDegrees() {
        return Math.toDegrees(targetAngle);
    }
    
    private double mapAngleToServoPosition(double angleRadians) {
        double angleDegrees = Math.toDegrees(angleRadians);
        double normalizedAngle = (angleDegrees - 0) / 90;
        double servoPosition = servoMinPosition + normalizedAngle * (servoMaxPosition - servoMinPosition);
        return Math.max(servoMinPosition, Math.min(servoMaxPosition, servoPosition));
    }
    
    public void stop() {
        if (!useServo) {
            angleMotor.setPower(0);
        }
    }
    
    public void reset() {
        if (useServo) {
            angleServo.setPosition(servo45Position);
            currentAngle = Math.PI / 4;
        } else {
            angleMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            angleMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            currentAngle = 0;
            anglePID.reset();
        }
        targetAngle = Math.PI / 4;
    }
    
    public void updateTelemetry() {
        telemetry.addData("Turret Current Angle", String.format("%.2f°", getCurrentAngleDegrees()));
        telemetry.addData("Turret Target Angle", String.format("%.2f°", getTargetAngleDegrees()));
        telemetry.addData("Turret At Target", isAtTarget());
    }
}