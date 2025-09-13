package org.firstinspires.ftc.teamcode.controllers;

public class RobotPosition {
    private RobotPosition(){

    }
    private static RobotPosition instance;
    public static RobotPosition getInstance(){
        if(instance==null){
            instance=new RobotPosition();
        }
        return instance;
    }
    public Point3D position=new Point3D(0,0,0);
    public double headingRadian=0;
}
