package org.firstinspires.ftc.teamcode.controller;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.utilities.FieldColor;
import java.util.ArrayList;

public class AprilTagController extends Controller<Double>{

    protected final AprilTagProcessor processor;
    private final Drive drive;
    private double tagID;
    private double lastReadingTime = 0;
    protected double cameraOffset = FieldColor.getActive() == FieldColor.BLUE ? 6.5 :3.5;
    public double error;
    public double motorSpeed;
    public double offSet;
    private static final ElapsedTime clock = new ElapsedTime(); // to maintain time
    public AprilTagController( Drive drive,AprilTagProcessor processor) {
        this.processor = processor;
        this.drive = drive;
    }

    @Override
    public boolean run() {
         return false;
    }
    
    public double getError(){
        offSet = getCurrentSensorValue();
        error = offSet - target;
        if(FieldColor.getActive() == FieldColor.BLUE)error = -error;
        return error;
    }

    @Override
    public Double getCurrentSensorValue() {
        ArrayList<AprilTagDetection> detections = processor.getDetections();
        for (AprilTagDetection detection : detections) {
            if(detection.metadata != null){
                    int primeId = FieldColor.getActive() == FieldColor.BLUE ? 3:6 ;
                    return detection.ftcPose.x + cameraOffset + ((primeId-detection.id)*6);
                
            }
        }
        return -1d;

    }
    
    public void setAdditionalInformation(double tagID){
        this.tagID = tagID;
    }
    
    protected static double calculateMotorSpeed(double error) {
        double absoluteError = Math.abs(error);
        double speed = absoluteError > 3 ? 0.2 : 0.01* absoluteError;
        speed = Math.max(0.19, speed);
        double motorSpeed = Math.copySign(speed, error);
        return motorSpeed;
    }


}
