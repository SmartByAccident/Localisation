package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Datalogger;

import java.util.concurrent.TimeUnit;

public final class SplineTest extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        ElapsedTime clock = new ElapsedTime(0);
        long start = clock.now(TimeUnit.MILLISECONDS);
        Datalog datalog = new Datalog("datalog_01.csv");
        while(opModeIsActive()){
            telemetry.addData("FL",leftFront.getCurrentPosition());
            telemetry.addData("FR",rightFront.getCurrentPosition());
            telemetry.addData("BR",rightBack.getCurrentPosition());
            telemetry.addData("BL",leftBack.getCurrentPosition());
            datalog.PIDOutput.set(clock.now(TimeUnit.MILLISECONDS)-start);
            datalog.elbowPosition.set(leftBack.getCurrentPosition());
            datalog.opModeStatus.set(leftFront.getCurrentPosition());
            datalog.target.set(rightBack.getCurrentPosition());
            datalog.loopCounter.set(rightFront.getCurrentPosition());
            datalog.writeLine();
            if(clock.now(TimeUnit.MILLISECONDS)-start<=10000){
            leftFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightFront.setPower(0.3);
            rightBack.setPower(0.3);
            telemetry.addData("Running",clock.now(TimeUnit.MILLISECONDS)-start);
            }else{
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
                telemetry.addData("Not Running",clock.now(TimeUnit.MILLISECONDS)-start);


            }
            telemetry.update();
        }
    }
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField target      = new Datalogger.GenericField("target");
        public Datalogger.GenericField elbowPosition      = new Datalogger.GenericField("Elbow Position");
        public Datalogger.GenericField PIDOutput      = new Datalogger.GenericField("PIDOutput");
        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            elbowPosition,
                            target,
                            PIDOutput

                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }

}
