package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.utilities.PIDCalculator;
import org.firstinspires.ftc.teamcode.utilities.FieldColor;
public class IMUController extends Controller<Double> {
    private final IMU imu;
    private final Drive drive;
    private final PIDCalculator pidCalculator= new PIDCalculator();


    public IMUController(IMU imu, Drive drive) {
        this.imu = imu;
        this.drive = drive;
        // pidCalculator.set(0.07,0.01,0.17);
    }

    public double getRotationPower(){
        double localTarget = target;
         if(FieldColor.getActive() == FieldColor.BLUE) localTarget = -target;
        double currentAngle = getCurrentSensorValue();
        // double rotationPower = 
        pidCalculator.compute(currentAngle, localTarget);
        double pidOutput = pidCalculator.output;
        double reduction = Math.abs(localTarget - currentAngle)> 30 ? 0.3 : 0.2;//1;

        double rotationPower = pidOutput * reduction;
        double absolutePower=Math.abs(rotationPower);
        if(absolutePower < 0.12)
            absolutePower = 0.12;
            
        if(rotationPower<0)
        absolutePower*=-1;
        rotationPower=absolutePower;
        if (Math.abs(currentAngle-localTarget) < 2) rotationPower = 0;  
    
        return rotationPower;
    }
    @Override
    public boolean run() {
        double localTarget = target;
        if(FieldColor.getActive() == FieldColor.BLUE) localTarget = -target;
        double currentAngle = getCurrentSensorValue();
        drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double rotationPower = getRotationPower();
            drive.rotate(rotationPower);
        

        return Math.abs(currentAngle-localTarget) < 2;
        
        
    }

    @Override
    public Double getCurrentSensorValue() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


}
