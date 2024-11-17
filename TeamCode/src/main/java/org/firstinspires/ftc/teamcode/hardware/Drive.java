package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

/**
 * This is an utility class so that we can control the motors of the drive abstractly.
 */
public class Drive {
    public final List<DcMotorImplEx> motors;

    public Drive(List<DcMotorImplEx> motors) {
        this.motors = motors;
        
        correctDirection();
        motors.forEach(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        motors.forEach(motor -> motor.setPositionPIDFCoefficients(4.5d));
       
        
        
    }
    /**
     * Sets the correct direction for the motors. (This is needed because the direction for the left hand motors are flipped tdue to thier placement in the hardware)
     */
    public void correctDirection() {
        motors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(2).setDirection(DcMotorSimple.Direction.REVERSE);
    }
    

    public void setMotorMode(DcMotor.RunMode mode){
        motors.forEach(motor -> motor.setMode(mode));
    }

    public void rotate(double rotationPower){
        this.setPower(rotationPower,-rotationPower);
    }

    public void setPower(double diagonal1, double diagonal2) {
        motors.get(0).setPower(diagonal1);
        motors.get(2).setPower(diagonal1);

        motors.get(1).setPower(diagonal2);
        motors.get(3).setPower(diagonal2);
    }
    public void setDiagonalPower(double diagonal1, double diagonal2) {
        motors.get(0).setPower(diagonal2);
        motors.get(3).setPower(diagonal2);
        
        motors.get(1).setPower(diagonal1);
        motors.get(2).setPower(diagonal1);
    }
    
    public void setDiagonalPower(double diagonal1, double diagonal2, double rotate) {
        motors.get(0).setPower(diagonal2 + rotate);
        motors.get(3).setPower(diagonal2 - rotate);
        
        motors.get(1).setPower(diagonal1 - rotate);
        motors.get(2).setPower(diagonal1 + rotate);
    }
    
    public void setPower(double power) {
        motors.forEach(motor -> motor.setPower(power));
    }
}

