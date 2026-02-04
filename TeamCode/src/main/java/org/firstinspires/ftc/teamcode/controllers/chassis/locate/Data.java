/**
 * 机器人数据类
 * 负责存储和管理机器人的位置、速度、朝向等数据
 */
package org.firstinspires.ftc.teamcode.controllers.chassis.locate;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;
@Config
public class Data {
    /**
     * 私有构造函数，防止外部实例化
     */
    private Data(){
    }
    
    /**
     * 重写toString方法，返回数据的字符串表示
     * @return 数据的字符串表示
     */
    @Override
    public String toString(){
        return "Position:\nx:"+getPosition(DistanceUnit.MM).getX()+
                "\ny:"+getPosition(DistanceUnit.MM).getY()+
                "\nHeadingRadian:\n"+headingRadian+
                "\nHeadingSpeedRadianPerSec:\n"+headingSpeedRadianPerSec+
                "\nSpeed:\nX:"+getSpeed(DistanceUnit.MM).x+
                "\nY:"+getSpeed(DistanceUnit.MM).y;
    }
    
    static Data instance=new Data(); // 单例实例
    
    /**
     * 获取Data实例
     * @return Data实例
     */
    public static Data getInstance(){return instance;}
    
    private Point2D position=new Point2D(0,0); // 机器人位置（英寸）
    
    /**
     * 设置机器人位置
     * @param positionInch 位置（英寸）
     */
    public void setPosition(Point2D positionInch){
        this.position=new Point2D(positionInch);
    }
    
    /**
     * 获取机器人位置
     * @param distanceUnit 距离单位
     * @return 位置
     */
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
    
    public double headingRadian=0; // 机器人朝向（弧度）
    public double headingSpeedRadianPerSec=0; // 机器人角速度（弧度/秒）
    private Vector2d speed=new Vector2d(0,0); // 机器人速度（英寸/秒）
    
    /**
     * 设置机器人速度
     * @param speed 速度
     */
    public void setSpeed(Vector2d speed){
        this.speed = speed;
    }
    
    /**
     * 获取机器人速度
     * @param distanceUnit 距离单位
     * @return 速度
     */
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
    
    /**
     * 获取Pose2d对象
     * @return Pose2d对象
     */
    public Pose2d getPose2d(){
        return new Pose2d(getPosition(DistanceUnit.INCH).getY(),-getPosition(DistanceUnit.INCH).getX(),headingRadian);
    }
    
    /**
     * 设置Pose2d对象
     * @param pose2d Pose2d对象
     */
    public void setPose2d(Pose2d pose2d){
        setPosition(new Point2D(-pose2d.position.y,+pose2d.position.x));
        headingRadian=pose2d.heading.toDouble();
    }
}
