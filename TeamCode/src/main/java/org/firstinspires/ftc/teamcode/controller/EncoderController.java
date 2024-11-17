package org.firstinspires.ftc.teamcode.controller;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.utilities.FieldColor;
public class EncoderController extends Controller<Pair<Double,Double>> {
    private static final ElapsedTime clock = new ElapsedTime(); // to maintain time
    // public static final double DEFAULT_MOTOR_POWER = 0.6;
    private final Drive drive;
    private final double motorPower;
    private final Odometry odometry;
    private double newMotorPower = 0;
    private boolean boostSpeed;


    public EncoderController(Drive drive, double motorPower, Odometry odometry) {
        this.drive = drive;
        this.motorPower = motorPower;
        this.odometry = odometry;
    }

    @Override
    public boolean run() {
        drive.setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //setPowersInRatio();
        double actualPower = newMotorPower > 0 ? newMotorPower : motorPower;
        if(boostSpeed)
            actualPower = actualPower*0.1;
        drive.setPower(actualPower);
        return  targetPositionReached();
    }
    
    public boolean targetPositionReached(){
       int sum = 0;
       for(int motorIndex=0;motorIndex<4;motorIndex++){
          if(! drive.motors.get(motorIndex).isBusy())
                sum = sum+1;
       }
       
       return sum > 2;
    }
    
    public boolean odometryCheck(){
        if(target.first > target.second){
            return (Math.abs(odometry.getCurrentPositionX() - target.first) < 1);
        }
        return Math.abs(odometry.getCurrentPositionY() - target.second) < 1;
    }

    private double calculateTotalDistance() {
        double xSquare = (target.first) * (target.first);
        double ySquare = (target.second) * (target.second);

        return Math.sqrt( xSquare +ySquare);
    }

    @Override
    public Pair<Double, Double> getCurrentSensorValue() {
        return null;
    }
    
    private void setPowersInRatio(){
        int diagonal2Target = drive.motors.get(0).getTargetPosition();//400
        int diagonal1Target = drive.motors.get(1).getTargetPosition();//300
        double maximumPower = 0.01;
        double diagonal2Power = (diagonal2Target/Math.max(diagonal2Target,diagonal1Target))*maximumPower;//0.3
        double diagonal1Power = (diagonal1Target/Math.max(diagonal2Target,diagonal1Target))*maximumPower;//0.225
        drive.setDiagonalPower(diagonal1Power,diagonal2Power);
    }

    /**
     * This method is used to set the targetValues to the motors
     * @param xEncoderTarget
     * @param yEncoderTarget
     */
    void setMotorTargets(int xEncoderTarget , int yEncoderTarget)
    {
        // newMotorPower = 0;
        if(FieldColor.getActive() == FieldColor.RED)xEncoderTarget = -xEncoderTarget;
    
        double angle = Math.toDegrees( Math.atan2(yEncoderTarget,xEncoderTarget));
        double target =Math.sqrt(2) * Math.sqrt((xEncoderTarget*xEncoderTarget) + (yEncoderTarget *yEncoderTarget));
        
        double diagonal1 = - target * Math.sin(Math.toRadians(angle+45));
        double diagonal2 = target * Math.cos(Math.toRadians(angle+45));

        int diagonal1Target = (int) Math.round(diagonal1);
        int diagonal2Target = (int) Math.round(diagonal2);


        drive.motors.get(0).setTargetPosition(diagonal2Target);
        drive.motors.get(1).setTargetPosition(diagonal1Target);
        drive.motors.get(2).setTargetPosition(diagonal1Target);
        drive.motors.get(3).setTargetPosition(diagonal2Target);

    }

    @Override
    public void setTarget(Pair<Double, Double> target) {
        super.setTarget(target);
        Double targetX = target.first;
        Double targetY = -1 * target.second;//multiply by -1 for direction
        Double yInchToTicksMultiplier = 24.64;
        Double xInchToTicksMultiplier = 28.83;
        int xEncoderTarget = (int) (targetX * xInchToTicksMultiplier);
        int yEncoderTarget = (int) (targetY * yInchToTicksMultiplier ); 
        
        drive.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometry.reset();
        setMotorTargets(xEncoderTarget,yEncoderTarget);
        
        boostSpeed = (calculateTotalDistance() < 14);


    }

    @Override
    public double maximumSecondsRequiredToReachTarget() {
        double totalDistance = calculateTotalDistance();
        return totalDistance;
    }
    
     public void setAdditionalInformation(double newMotorSpeed){
        this.newMotorPower = newMotorSpeed;
    }
    public String toString(){
        String data ="";
        String dataError ="";
        for(int i = 0; i< 4;i++){
            data = data + (drive.motors.get(i).getCurrentPosition()) + ",";
            dataError = dataError + (drive.motors.get(i).getTargetPosition() - drive.motors.get(i).getCurrentPosition()) + ",";
        }
        data = data + dataError + ",";
        data = data + (odometry.getCurrentPositionX()) + ",";
        data = data + (odometry.getCurrentPositionY()) + ",";
        data = data + (target.first) + "," + (target.second);
        // data = data + "/n";
        return data;
    }
   
}
