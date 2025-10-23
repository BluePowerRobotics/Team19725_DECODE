package org.firstinspires.ftc.teamcode.controllers.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;

public class ElevatorController {
    PIDController pidController;
    DcMotorEx elevatorMotor;
    public ElevatorController(HardwareMap hardwareMap){
        elevatorMotor=hardwareMap.get(DcMotorEx.class,"elevatorMotor");
        elevatorMotor.setDirection(DcMotorEx.Direction.FORWARD);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pidController=new PIDController(0.1,0,0.01,1);
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
