package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Drive;

public class DistanceController extends Controller<Double>{
    private DistanceSensor distanceSensor;
    private final Drive drive;
    private String sensor;
    private final HardwareMap hardwareMap;

    int records = 5;
    double motorSpeed = 0d;

    public DistanceController(Drive drive,HardwareMap hardwareMap) {
        this.drive = drive;
        this.hardwareMap= hardwareMap;
    }
    

    @Override
    public boolean run() {
       double error;
       Double sensorValue = getCurrentSensorValue();
       error = sensorValue - target;
       double motorSpeed = calculateMotorSpeed(error);
       double speed= calculateMotorSpeed(error);
       
        if(sensor=="DISTANCEL"){
            drive.setDiagonalPower(-motorSpeed , motorSpeed);
            
            
       }else if(sensor=="DISTANCER" ){
            drive.setDiagonalPower(motorSpeed , -motorSpeed);
    }
        else{
            drive.setDiagonalPower(motorSpeed, motorSpeed);
        }
        
     return Math.abs(error) < 3;   
    }

    @Override
    public Double getCurrentSensorValue() {/*
        Double sum = 0d;
    
        for (int count = 0; count < records; count++) {
         double value = distanceSensor.getDistance(DistanceUnit.INCH);
          sum =  (sum + value);
        }
        
        return sum / records;*/
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public void setAdditionalInformation(DistanceSensor distanceSensor){
        setAdditionalInformation(distanceSensor, 0d);
    }
     public void setAdditionalInformation(String distanceSensor){
         sensor = distanceSensor;
        setAdditionalInformation(hardwareMap.get(DistanceSensor.class,distanceSensor), 0d);
    }
    public void setAdditionalInformation(DistanceSensor distanceSensor, double speed){
        this.distanceSensor = distanceSensor;
        this.motorSpeed = speed;
    }
    
    protected double calculateMotorSpeed(double error) {
        if(motorSpeed != 0){return Math.copySign(motorSpeed, error);}
        double absoluteError = Math.abs(error);
        double speed = absoluteError > 3 ? 0.22 : 0.01* absoluteError;
        speed = Math.max(0.07, speed);
        double motorSpeed = Math.copySign(speed, error);
        return motorSpeed;
    }
    
    
}
