package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

@Autonomous
public class IntoTheDeepBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-17.1, 6.9, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Servo wrist = hardwareMap.get(Servo.class, "Wrist");
        Servo gripper = hardwareMap.get(Servo.class , "Gripper");
        DcMotor sliderMotor = hardwareMap.get(DcMotor.class , "ArmSlider");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "MainArm");
        RobomindsActions actionPack = new RobomindsActions(wrist,gripper,sliderMotor,pivotMotor);
        waitForStart();
        Vector2d observationDeck = new Vector2d(66,26.2);
        Vector2d submersible = new Vector2d(13.8,30.4);


        Actions.runBlocking(
                new SequentialAction(
                        actionPack.moveSliderToPosition(100),
                        actionPack.moveWristToPosition(0.8),
                        actionPack.moveGripperToPosition(0.12),
                        new ParallelAction(
                        actionPack.moveSliderToPosition(1400),
                        drive.actionBuilder(beginPose)
                            .strafeToLinearHeading(new Vector2d(-39.3,48),Math.toRadians(180))
                            .build()),

                        actionPack.moveGripperToPosition(0.587),
                        new SleepAction(0.1),
                        actionPack.moveSliderToPosition(100),
                        drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(new Vector2d(-58,20), Math.toRadians(225))
                            .build(),
                        actionPack.moveGripperToPosition(0.12),

                        new SleepAction(0.1),
                        new ParallelAction(
                                actionPack.moveSliderToPosition(1400),
                                drive.actionBuilder(beginPose)
                                .strafeToLinearHeading(new Vector2d(-51.3,48), Math.toRadians(180))
                                .build()),

                        actionPack.moveGripperToPosition(0.587),
                        new SleepAction(0.1),
                        actionPack.moveSliderToPosition(100),
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(-58,20), Math.toRadians(225))
                                .build(),
                        actionPack.moveGripperToPosition(0.12),


                        new SleepAction(0.1),
                        new ParallelAction(
                                actionPack.moveSliderToPosition(1400),
                                drive.actionBuilder(beginPose)
                                        .strafeToLinearHeading(new Vector2d(-61.3,48), Math.toRadians(180))
                                        .build())

//                        actionPack.moveGripperToPosition(0.587),
//                        new SleepAction(0.1),
//                        actionPack.moveSliderToPosition(100),
//                        drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(new Vector2d(-58,20), Math.toRadians(225))
//                                .build(),
//                        actionPack.moveGripperToPosition(0.12),
//                        new SleepAction(0.1)



                )
        );


    }
}
