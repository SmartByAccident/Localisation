package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor MainArm = hardwareMap.get(DcMotorEx.class,"MainArm");
        MainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MainArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            MainArm.setTargetPosition(100);
            MainArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MainArm.setPower(0.8);


        }

    }


}
