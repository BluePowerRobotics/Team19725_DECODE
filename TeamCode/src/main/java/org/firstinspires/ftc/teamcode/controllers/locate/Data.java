package org.firstinspires.ftc.teamcode.controllers.locate;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public class Data {
    private Data(){}
    static Data instance=new Data();
    public Point2D position=new Point2D(0,0);
    public double headingRadian=0;
    public double headingSpeedRadianPerSec=0;
    public Vector2d speed=new Vector2d(0,0);
}
