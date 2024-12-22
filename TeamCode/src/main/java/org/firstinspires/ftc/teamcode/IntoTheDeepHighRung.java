package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class IntoTheDeepHighRung extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-17.1, 6.9, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Servo wrist = hardwareMap.get(Servo.class, "Wrist");
        Servo gripper = hardwareMap.get(Servo.class , "Gripper");
        DcMotor sliderMotor = hardwareMap.get(DcMotor.class , "ArmSlider");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "MainArm");
        RobomindsActions actionPack = new RobomindsActions(wrist,gripper,sliderMotor,pivotMotor);
        gripper.setPosition(0.62);
        waitForStart();
        Vector2d observationDeck = new Vector2d(66,26.2);
        Vector2d submersible = new Vector2d(13.8,30.4);

        //Pivot :390 , SLider:750 ->100
        Actions.runBlocking(
                new SequentialAction(
//                        actionPack.moveGripperToPosition(0.62),
                        drive.actionBuilder(beginPose).strafeTo(new Vector2d(-17.1,24.4)).build(),
                        new SleepAction(0.2),
                        actionPack.moveWristToPosition(0.1),
                        actionPack.movePivotToPosition(555),
                        actionPack.moveSliderToPosition(1300),
                        drive.actionBuilder(beginPose).strafeTo(new Vector2d(-17.1, 41.1), (pose2dDual, posePath, v) -> 30).build(),

//                        drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90))).strafeTo(new Vector2d(0,5)).build(),
//                        actionPack.moveSliderToPosition(100),
//                        actionPack.moveWristToPosition(1),
//                        new SleepAction(0.2),
                        actionPack.moveSliderToPosition(100),
                        actionPack.moveGripperToPosition(0.12),
                        new SleepAction(0.2)

                )
        );


    }
}
