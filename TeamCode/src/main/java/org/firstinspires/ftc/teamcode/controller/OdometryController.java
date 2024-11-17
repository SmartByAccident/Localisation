package org.firstinspires.ftc.teamcode.controller;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.utilities.FieldColor;
public class OdometryController extends Controller<Pair<Double,Double>> {
    private static final ElapsedTime clock = new ElapsedTime(); // to maintain time
    // public static final double DEFAULT_MOTOR_POWER = 0.6;
    private final Drive drive;
    private final double motorPower;
    private final Odometry odometry;
    private double newMotorPower = 0;
    private boolean boostSpeed;
    private boolean checkX = true;
    private double kp = 0.0255;


    public OdometryController(Drive drive, double motorPower, Odometry odometry) {
        this.drive = drive;
        this.motorPower = motorPower;
        this.odometry = odometry;
        this.setTarget(new Pair<>(0d,0d));
    }

    @Override
    public boolean run() {
        Pair<Double,Double> currentValue = getCurrentSensorValue();
        Pair<Double,Double> errorValue = getErrorValue(currentValue);
        Pair<Double,Double> newSpeed = getSpeedValue(errorValue);
        
        double xSpeed = newSpeed.first;
        double ySpeed = - newSpeed.second;
        
        double angle = Math.toDegrees( Math.atan2(ySpeed,xSpeed));
        double speed = Math.sqrt(2) * Math.sqrt((ySpeed*ySpeed) + (xSpeed *xSpeed));

        double diagonal1 = -speed * Math.sin(Math.toRadians(angle+45));
        double diagonal2 = speed * Math.cos(Math.toRadians(angle+45));
        
        
        drive.setDiagonalPower(diagonal1,diagonal2);
        
        return odometryCheck();
    }
    
    @Override
    public Pair<Double,Double> getCurrentSensorValue(){
        return new Pair<Double,Double>(odometry.getCurrentPositionX(),odometry.getCurrentPositionY());
    }
    
     private Pair<Double,Double> getErrorValue(Pair<Double,Double> currentValue ){
        double xError =  target.first - currentValue.first ;
        double yError =  target.second - currentValue.second;
        
        return new Pair<Double,Double>(-xError,yError);
    }
    
    private Pair<Double,Double> getSpeedValue(Pair<Double,Double> errorValue ){
      
        double xError =  errorValue.first ;
        double yError =  errorValue.second;
        
        double xSpeed = getPower(xError);
        double ySpeed = getPower(yError);
        
        return new Pair<Double,Double>(xSpeed,ySpeed);
    }
    
    private double getPower(double error){
        if(Math.abs(error)< 0.2 )
          return 0;
        error = error * kp;
        return Math.copySign( Range.clip(Math.abs(error) , 0.09 , motorPower) , error);
    }
    
    public void setTarget(Pair<Double,Double> target){
        
        
        if(Math.abs(target.first) > Math.abs(target.second)){
            double targetXError = FieldColor.getActive() == FieldColor.RED ? target.first : - target.first;
            target = new Pair<Double,Double>(targetXError,0d);
            checkX = true;
        }
        else{
            target = new Pair<Double,Double>(0d,target.second);
            checkX = false;
        }
        
        super.setTarget(target);
        odometry.reset();
        drive.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
    }
    
     public void updateTarget(Pair<Double,Double> target){
        
         
        if(Math.abs(target.first) > Math.abs(target.second)){
            double targetXError = FieldColor.getActive() == FieldColor.RED ? target.first : - target.first;
            target = new Pair<Double,Double>(this.target.first + targetXError, this.target.second);
            checkX = true;
            
        }
        else{
            target = new Pair<Double,Double>(this.target.first,this.target.second + target.second);
            checkX = false;
        }
        
        super.setTarget(target);
        drive.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
    }
    
     public boolean odometryCheck(){
        if(checkX){
            return (Math.abs(odometry.getCurrentPositionX() - target.first) < 0.2);
        }
        return Math.abs(odometry.getCurrentPositionY() - target.second) < 0.2;
    }
    
    public String toString(){
        Pair<Double,Double> currentSensorValue = getCurrentSensorValue();
        Pair<Double,Double> error = getErrorValue(currentSensorValue);
        
        String data =String.format("%2f",error.first) + "," ;
        String dataError =String.format("%2f",error.second) + ",";
        data = data + dataError;
        data = data + String.format("%2f",currentSensorValue.first) + ",";
        data = data + String.format("%2f",currentSensorValue.second) + ",";
        data = data + (target.first) + "," + (target.second);
        // data = data + "/n";
        return data;
    }
    public void setAdditionalInformation(double kp){
        this.kp = kp;
    }
    
    
   
   
}
