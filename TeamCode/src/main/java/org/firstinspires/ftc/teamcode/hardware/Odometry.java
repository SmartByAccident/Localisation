package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Odometry {
    
    DcMotorEx motorX;
    DcMotorEx motorY;
    
    //Kaushik was here
    public Odometry(DcMotorEx motorX, DcMotorEx motorY){
        this.motorX = motorX;
        this.motorY = motorY;
        reset();
    }
    
    public void reset(){
        motorX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double getCurrentPositionX(){
        return -motorX.getCurrentPosition() / 343.5;
        
    }
    public double getCurrentPositionY(){
        return -motorY.getCurrentPosition() / 343.5;
    }
}