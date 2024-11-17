package org.firstinspires.ftc.teamcode.controller;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;


public class FieldAprilTagController extends AprilTagController{
    Double yError = 0d;

    public FieldAprilTagController(Drive drive,AprilTagProcessor processor){
        super(drive, processor);
    }
    
    public Double getYError(){
        return yError;
    }
    
    @Override
    public Double getCurrentSensorValue() {
        ArrayList<AprilTagDetection> detections = processor.getDetections();
        for (AprilTagDetection detection : detections) {
            if(detection.metadata != null){
                   yError = detection.ftcPose.y;
                    if(detection.id == 8){
                        
                        return detection.ftcPose.x;
                    }
                    
                    //return detection.ftcPose.x;
                
            }
        }
        return -1d;

    }
    
    
}
