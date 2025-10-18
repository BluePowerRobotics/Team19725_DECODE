package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;


public class PIDController {
    private double kP, kI, kD;
    private double integral, previousError;
    private double maxI=1;
    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 1);
    }
    public PIDController(double kP, double kI, double kD, double maxI) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxI = maxI;
        this.integral = 0;
        this.previousError = 0;
    }

    public double calculate(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        integral += error * dt;
        // 积分限幅
        if (integral > maxI) {
            integral = maxI;
        } else if (integral < -maxI) {
            integral = -maxI;
        }
        double derivative = (error - previousError) / dt;

        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
    public void setPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public void setMaxI(double maxI) {
        this.maxI = maxI;
    }
}
