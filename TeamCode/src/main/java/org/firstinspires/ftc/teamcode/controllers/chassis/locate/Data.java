package org.firstinspires.ftc.teamcode.controllers.chassis.locate;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;
@Config
public class Data {
    private Data(){}
    @Override
    public String toString(){
        return "Position:\nx:"+getPosition(DistanceUnit.MM).getX()+
                "\ny:"+getPosition(DistanceUnit.MM).getY()+
                "\nHeadingRadian:\n"+headingRadian+
                "\nHeadingSpeedRadianPerSec:\n"+headingSpeedRadianPerSec+
                "\nSpeed:\nX:"+getSpeed(DistanceUnit.MM).x+
                "\nY:"+getSpeed(DistanceUnit.MM).y;
    }
    static Data instance=new Data();
    public static Data getInstance(){return instance;}
    private Point2D position=new Point2D(0,0);
    public void setPosition(Point2D positionInch){
        this.position=new Point2D(positionInch);
    }
    public Point2D getPosition(DistanceUnit distanceUnit){
        switch (distanceUnit) {
            case METER:
                return new Point2D(MathSolver.toMM(position.getX())/1000,MathSolver.toMM(position.getY())/1000);
            case CM:
                return new Point2D(MathSolver.toMM(position.getX())/10,MathSolver.toMM(position.getY())/10);
            case MM:
                return new Point2D(MathSolver.toMM(position.getX()),MathSolver.toMM(position.getY()));
            case INCH:
                return position;
        }
        return null;
    }
    public double headingRadian=0;
    public double headingSpeedRadianPerSec=0;
    private Vector2d speed=new Vector2d(0,0);
    public void setSpeed(Vector2d speed){
        this.speed = speed;
    }
    public Vector2d getSpeed(DistanceUnit distanceUnit){
        switch (distanceUnit) {
            case METER:
                return new Vector2d(MathSolver.toMM(speed.x)/1000,MathSolver.toMM(speed.y)/1000);
            case CM:
                return new Vector2d(MathSolver.toMM(speed.x)/10,MathSolver.toMM(speed.y)/10);
            case MM:
                return new Vector2d(MathSolver.toMM(speed.x),MathSolver.toMM(speed.y));
            case INCH:
                return speed;
        }
        return null;
    }
    public Pose2d getPose2d(){
        return new Pose2d(getPosition(DistanceUnit.INCH).getY(),-getPosition(DistanceUnit.INCH).getX(),headingRadian);
    }
    public void setPose2d(Pose2d pose2d){
        setPosition(new Point2D(-pose2d.position.y,+pose2d.position.x));
        headingRadian=pose2d.heading.toDouble();
    }
}
