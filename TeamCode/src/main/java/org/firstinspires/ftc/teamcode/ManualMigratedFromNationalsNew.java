package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.Drive;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;


@TeleOp


public class ManualMigratedFromNationalsNew extends OpMode {

    Drive driveTrain;
    DcMotor pivotMotor;
    DcMotor sliderMotor;
    Servo gripper;
    Servo wrist;
    Servo swivel;
    Button sliderHighBasket;
    Button sliderHome;
    Button sliderPickup;
    Button sliderMotorModePressed;
    Button sliderMotorModeReleased;
    Button pivotControl;
    Button gripperToggle;
    Button wristToggle;
    VoltageSensor voltageSensor;
    Datalog datalog;


    void move() {
        double limit = 0.80;
        double speed_y = limit * this.gamepad1.left_stick_y;
        double speed_x = -limit * this.gamepad1.left_stick_x;
        double speed_rotate = -0.6 * this.gamepad1.right_stick_x;


        if (this.gamepad1.right_trigger > 0.3) {
            speed_y = speed_y * 0.416666666;
            speed_x = speed_x * 0.416666666;
            speed_rotate = speed_rotate * 0.416666666;
        } else if (this.gamepad1.right_bumper) {
            speed_y = speed_y * 0.25;
            speed_x = speed_x * 0.25;
            speed_rotate = speed_rotate * 0.25;
        }

        double angle = Math.toDegrees(Math.atan2(speed_y, speed_x));
        double speed = Math.sqrt(2) * Math.sqrt((speed_y * speed_y) + (speed_x * speed_x));

        double diagonal1 = -speed * Math.sin(Math.toRadians(angle + 45));
        double diagonal2 = speed * Math.cos(Math.toRadians(angle + 45));





        driveTrain.setDiagonalPower(diagonal1, diagonal2, speed_rotate );


    }

    private void telemetry() {
        telemetry.addData("Servo pos", wrist.getPosition());
        telemetry.addData("button", wristToggle.lastButtonState);
        telemetry.addData("WristPosition", wrist.getPosition());
        telemetry.addData("ModeSlider" , sliderMotor.getMode());
        telemetry.addData("SLider Position" , sliderMotor.getCurrentPosition());
        telemetry.update();
    }

    void arm() {
        if (pivotControl.onPressed()) {
            int armTarget = pivotMotor.getTargetPosition() == 0 ? 420: 0;
            pivotMotor.setTargetPosition(armTarget);

        }
        if (gripperToggle.onPressed()) {
            double currentPosition = gripper.getPosition();
            double newPosition = Math.abs(currentPosition - 0.12) < 0.01 ? 0.587: 0.12;
            gripper.setPosition(newPosition);
        }
        if (wristToggle.onPressed()) {

            double currentPosition = wrist.getPosition();
            double newPosition = Math.abs(currentPosition - 1) < 0.01 ? 0.1 : 1;
            wrist.setPosition(newPosition);
        }


        double value = sliderMotor.getCurrentPosition();
        if (sliderMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            if (value > 10000)
                sliderMotor.setPower(Math.min(0, -gamepad2.left_stick_y * 0.4));

            else if (value < 100)
                sliderMotor.setPower(Math.max(0, -gamepad2.left_stick_y * 0.4));
            else {
                sliderMotor.setPower(-gamepad2.left_stick_y * 0.4);
            }
        }

        if (sliderPickup.onPressed()) {

            sliderMotor.setTargetPosition(7000);
        }

        if (sliderHighBasket.onPressed()) {

            sliderMotor.setTargetPosition(10000);
            wrist.setPosition(0.75);
        }
        if(sliderHome.onPressed()) {
            sliderMotor.setTargetPosition(100);
//            wrist.setPosition(0.1);

        }
            if (sliderMotorModePressed.onPressed()) {
            sliderMotor.setTargetPosition(sliderMotor.getCurrentPosition());
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotor.setPower(0.95);
        }
        if (sliderMotorModeReleased.onReleased()) {
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }


    @Override
    public void init() {
        List<String> motorConfigs = Arrays.asList("rightFront", "leftFront", "rightBack", "leftBack");
        List<DcMotorImplEx> driveMotors = motorConfigs.stream().map(config -> hardwareMap.get(DcMotorImplEx.class, config)).collect(Collectors.toList());
        driveTrain = new Drive(driveMotors);
        driveTrain.setMotorMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor = hardwareMap.get(DcMotorEx.class, "MainArm");
//        MainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotControl = new Button(() -> gamepad2.a);
        gripperToggle = new Button(() -> gamepad2.left_bumper);
        gripper = hardwareMap.get(Servo.class, "Gripper");
        gripper.setPosition(0.6);
        wristToggle = new Button(() -> gamepad2.left_trigger > 0.5);
        sliderMotorModePressed = new Button(() -> gamepad2.right_bumper);
        sliderMotorModeReleased = new Button(() -> gamepad2.right_bumper);
        wrist = hardwareMap.get(Servo.class, "Wrist");
        wrist.setPosition(0.1);
        pivotMotor.setTargetPosition(0);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(0.3);
        sliderHighBasket = new Button(()->gamepad2.y);
        sliderPickup = new Button(()->gamepad2.x);
        sliderHome = new Button(() -> gamepad2.b);
        sliderMotor = hardwareMap.get(DcMotor.class, "ArmSlider");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        swivel = hardwareMap.get(Servo.class,"Swivel");
        swivel.setPosition(0.43);
//        ArmSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // ArmSlider.setTargetPosition(1000);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setPower(0);

        datalog = new Datalog("datalog_01.csv");


        //Init voltageSensor
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        //create controllers

        // DistanceController distanceController = new DistanceController(driveTrain);

    }

    @Override
    public void loop() {
        move();
        arm();
        telemetry();

    }

    @Override
    public void stop() {
    }

    public class Button {
        boolean lastButtonState;
        private final Supplier<Boolean> supplier;


        public Button(Supplier<Boolean> button) {
            supplier = button;
        }

        boolean onPressed() {
            Boolean currentState = supplier.get();
            //current state == true and lastButtonState == false that mean the button is just pressed
            boolean output = currentState && !lastButtonState;
            lastButtonState = currentState;
            return output;
        }

        boolean onReleased() {
            Boolean currentState = supplier.get();
            //current state == false and lastButtonState == true that mean the button is just released
            boolean output = !currentState && lastButtonState;
            lastButtonState = currentState;
            return output;
        }
    }

    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField battery = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField elbowPosition = new Datalogger.GenericField("Elbow Position");

        public Datalog(String name) {
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
                            battery

                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }


}

