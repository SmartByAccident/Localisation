package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class TestPlan extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();
        int waitSeconds = 45;
        Vector2d observationDeck = new Vector2d(66,26.2);
        Vector2d a6 = new Vector2d(0,120);
        Vector2d f6 = new Vector2d(100, 120);



        Actions.runBlocking(
                new ParallelAction(telemetryPacket -> {
                    double yaw = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    telemetryPacket.addLine(String.valueOf(yaw));
                    telemetryPacket.addLine(drive.pose.position.toString());
                    telemetryPacket.addLine(String.valueOf(Math.toDegrees(drive.pose.heading.log())));
                    return true;
                },

                drive.actionBuilder(beginPose)
                        .strafeTo(a6)
                        .waitSeconds(waitSeconds)
                        .strafeTo(f6)
                        .waitSeconds(waitSeconds)
                        .strafeTo(a6)
                        .waitSeconds(waitSeconds)
                        .turn(Math.toRadians(90))
                        .waitSeconds(waitSeconds)
                        .turn(Math.toRadians(-45))
                        .waitSeconds(waitSeconds)
                        .strafeToLinearHeading(f6,Math.toRadians(90))
                        .waitSeconds(waitSeconds)
                        .strafeToLinearHeading(a6,Math.toRadians(180))
                        .waitSeconds(waitSeconds)
                        .build()));
    }
}
