package org.firstinspires.ftc.teamcode.controllers.elevator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;
@Config
public class ElevatorController {
    PIDController pidController;
    DcMotorEx elevatorMotor;
    public static double kp=0.1,ki=0,kd=0.01,maxI=1;
    public ElevatorController(HardwareMap hardwareMap){
        elevatorMotor=hardwareMap.get(DcMotorEx.class,"elevatorMotor");
        elevatorMotor.setDirection(DcMotorEx.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pidController=new PIDController(kp,ki,kd,maxI);
    }
    public void setPower(double power){
        if(Double.isNaN(power)){
            return;
        }else if(Math.abs(power)>1){
            power= MathSolver.sgn(power);
        }
        elevatorMotor.setPower(power);
    }
    public void setPosition(int Encoder){
        double control = pidController.calculate(Encoder,elevatorMotor.getCurrentPosition(),0.02);
        elevatorMotor.setPower(control);
    }

}
