package org.firstinspires.ftc.teamcode.controllers.turret.model;

import androidx.annotation.NonNull;

import java.util.Locale;

public class TurretInfo {
    public double v;      // 初速度
    public double phi;    // 水平角 (弧度)
    public double theta;  // 仰角 (弧度)
    public double t;      // 飞行时间

    public TurretInfo(double v, double phi, double theta, double t) {
        this.v = v;
        this.phi = phi;
        this.theta = theta;
        this.t = t;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(
                Locale.CHINA,"v=%.3f m/s, phi=%.3f° (%.3f rad), theta=%.3f° (%.3f rad), t=%.3f s",
                v, Math.toDegrees(phi), phi, Math.toDegrees(theta), theta, t
        );
    }
}