package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobomindsActions {
    Servo wrist;
    DcMotor sliderMotor;
    DcMotor pivotMotor;
    Servo gripper;
    public RobomindsActions(Servo wrist,  Servo gripper, DcMotor sliderMotor, DcMotor pivotMotor){
        this.wrist = wrist;
        this.sliderMotor = sliderMotor;
        this.pivotMotor = pivotMotor;
        this.gripper = gripper;
    }
//
    
    public Action moveWristToPosition(double setPoint){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(setPoint);
                return false;
            }
        };
    }
    public Action moveGripperToPosition(double setPoint){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                gripper.setPosition(setPoint);
                return false;
            }
        };
    }

    public Action moveSliderToPosition(int setPoint){

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sliderMotor.setTargetPosition(setPoint);
                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderMotor.setPower(0.8);
                return sliderMotor.isBusy();
            }
        };
    }

    public Action movePivotToPosition(int setPoint){


        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivotMotor.setTargetPosition(setPoint);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(0.4);
                return pivotMotor.isBusy();
            }
        };
    }

    public Action grabElement(){
        return new SequentialAction(
                movePivotToPosition(100),
                new ParallelAction(
                    moveWristToPosition(0),
                    moveGripperToPosition(0.6),
                    moveSliderToPosition(4800)
                ),
                moveWristToPosition(0.6),
                moveSliderToPosition(50));
    }

//    public Action hangSpecimen(){
//        return new SequentialAction(
//                new ParallelAction(
//                    moveWristToPosition(0.1),
//                    movePivotToPosition(400)
//                ),
//                moveSliderToPosition(1500),
//
//        );
//    }




}
