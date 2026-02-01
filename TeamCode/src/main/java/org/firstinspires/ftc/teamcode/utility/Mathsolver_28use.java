package org.firstinspires.ftc.teamcode.utility;
import java.lang.Math;
import java.util.HashMap;
import java.util.Map;

public class Mathsolver_28use {
    public double solveVDistance (double solveDegree,double distance){
        return (1/Math.tan(Math.toRadians(solveDegree))) * distance;
    }
    public double solveGDistance (double solveDegree,double distance,double turningspeed,double radius){
        return 0.5*980.5*(distance/(turningspeed*radius*Math.sin(Math.toRadians(solveDegree))))*(distance/(turningspeed*radius*Math.sin(Math.toRadians(solveDegree))));
    }  
    public double solveHeight (double solveDegree,double distance,double turningspeed, double radius, double height){
        double heightt=height+radius*Math.sin(Math.toRadians(solveDegree));
        double length=radius*(1-Math.cos(Math.toRadians(solveDegree)));
        return solveVDistance(solveDegree,distance-length)-solveGDistance(solveDegree,distance-length,turningspeed,radius)+heightt;
    }
    public double solveShootingDegree(double x,double y,double turningspeed,double radius,double height){
        double aimx=182.88;
        double aimy=182.88;          
        //TODO: 修改此处瞄准点坐标 

        double aimdistance=(Math.sqrt((aimx-x)*(aimx-x)+(aimy-y)*(aimy-y)));
        int degree=0;
        double targetradius=12.94;
        //TODO: 修改此处瞄准中心半径
        double aimdistanceMax=aimdistance;
        double aimdistanceMin=aimdistance-targetradius;
        double aimheightMax=134.62;
        double aimheightMin=96.52;
        double bestdegree=0;
        double bestdelta=0;
        Map<Integer, Double> degrees = new HashMap<>();
        for(degree=0;degree<=90;degree++){
            if(solveHeight(degree,aimdistanceMin,turningspeed,radius,height)>=aimheightMin && solveHeight(degree,aimdistanceMax,turningspeed,radius,height)<=aimheightMax){
                if (bestdelta<=-(solveHeight(degree,aimdistanceMin,turningspeed,radius,height)-solveHeight(degree,aimdistanceMax,turningspeed,radius,height))){
                    bestdelta=-(solveHeight(degree,aimdistanceMin,turningspeed,radius,height)-solveHeight(degree,aimdistanceMax,turningspeed,radius,height));
                    bestdegree=degree;
                }
                degrees.put(degree,-(solveHeight(degree,aimdistanceMin,turningspeed,radius,height)-solveHeight(degree,aimdistanceMax,turningspeed,radius,height)));
            }
        }
        return bestdegree;

    }

    public Map<Integer,Double> solveShootingDegreeMAP(double x,double y,double turningspeed,double radius,double height){
        double aimx=144;
        double aimy=144;
        //TODO: 修改此处瞄准点坐标

        double aimdistance=(Math.sqrt((aimx-x)*(aimx-x)+(aimy-y)*(aimy-y)));
        int degree=0;
        double targetradius=12.94;
        //TODO: 修改此处瞄准中心半径
        double aimdistanceMax=aimdistance;
        double aimdistanceMin=aimdistance-targetradius;
        double aimheightMax=134.62;
        double aimheightMin=96.52;
        double bestdegree=0;
        double bestdelta=0;
        Map<Integer, Double> degrees = new HashMap<>();
        for(degree=0;degree<=90;degree++){
            if(solveHeight(degree,aimdistanceMin,turningspeed,radius,height)>=aimheightMin && solveHeight(degree,aimdistanceMax,turningspeed,radius,height)<=aimheightMax){
                if (bestdelta<=solveHeight(degree,aimdistanceMin,turningspeed,radius,height)-solveHeight(degree,aimdistanceMax,turningspeed,radius,height)){
                    bestdelta=solveHeight(degree,aimdistanceMin,turningspeed,radius,height)-solveHeight(degree,aimdistanceMax,turningspeed,radius,height);
                    bestdegree=degree;
                }
                degrees.put(degree,solveHeight(degree,aimdistanceMin,turningspeed,radius,height)-solveHeight(degree,aimdistanceMax,turningspeed,radius,height));
            }
        }

        return degrees;

    }


    public static void main(String[] args) {
        Mathsolver_28use solver = new Mathsolver_28use();
        double x = 0;  // Example x coordinate
        double y = 0;  // Example y coordinate
        double turningspeed = 75; // Example turning speed
        double radius = 10.16; // Example radius
        double height = 0; // Example height

        double shootingDegree = solver.solveShootingDegree(x, y, turningspeed, radius, height);
        System.out.println("Optimal Shooting Degree: " + shootingDegree);
        solver.solveShootingDegreeMAP(x, y, turningspeed, radius, height).forEach((key, value) -> System.out.println("键：" + key + "，值：" + value));
    }


}



