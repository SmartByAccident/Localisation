package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class IntoTheDeep extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(24, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();
        Vector2d observationDeck = new Vector2d(66,26.2);
        Vector2d submersible = new Vector2d(13.8,30.4);


        Actions.runBlocking(
                new ParallelAction(telemetryPacket -> {
                    telemetryPacket.addLine(String.valueOf(Math.toDegrees(drive.pose.heading.log())));
                    return true;
                },

                drive.actionBuilder(beginPose)
                        .strafeTo(submersible)
                        .waitSeconds(1)
                        .strafeTo(observationDeck)
                        .turnTo(Math.toRadians(120))
                        .waitSeconds(1)
                        .turnTo(Math.toRadians(-90))
                        .waitSeconds(1)
                        .turnTo(Math.toRadians(90))
                        .waitSeconds(1)
                        .turnTo(Math.toRadians(-90))
                        .waitSeconds(1)
                        .turnTo(Math.toRadians(60))
                        .waitSeconds(1)
                        .turnTo(Math.toRadians(-90))
                        .build()));
    }
}
