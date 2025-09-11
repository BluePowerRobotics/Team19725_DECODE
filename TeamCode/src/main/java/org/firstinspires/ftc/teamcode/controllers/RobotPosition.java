package org.firstinspires.ftc.teamcode.controllers;

public class RobotPosition {
    private RobotPosition(){

    }
    RobotPosition instance;
    public static RobotPosition getInstance(){
        if(getInstance().instance==null){
            getInstance().instance=new RobotPosition();
        }
        return getInstance().instance;
    }
    public Point3D position=new Point3D(0,0,0);
    public double headingRadian=0;
}
