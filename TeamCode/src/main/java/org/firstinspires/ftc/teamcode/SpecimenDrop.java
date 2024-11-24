package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class SpecimenDrop extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(24, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(13.8,30.4))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(13.8,30.4),Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(13.8,30.4),Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(13.8,30.4),Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(13.8,30.4),Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(13.8,30.4))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(13.8,30.4),Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(13.8,30.4),Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(13.8,30.4),Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(13.8,30.4),Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(66,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(60,10.2),Math.toRadians(-90))
                        .waitSeconds(0.5)

                        .build());
    }
}
