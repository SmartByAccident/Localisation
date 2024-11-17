package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.controller.*;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;


@TeleOp


public class ManualMigratedFromNationalsNew extends LinearOpMode{
    

    IMU imu;
    IMUController imuController;
    Drive driveTrain;
    VoltageSensor voltageSensor;


 Datalog datalog;
    public void runOpMode(){
        initManualBot();
        waitForStart();
        while(must()){
            move();
            // telemetry.update();
        }
    }

    
    void move()
    {
        double limit = 0.80;
        double speed_y = limit*this.gamepad1.left_stick_y;
        double speed_x = -limit *this.gamepad1.left_stick_x;
        double speed_rotate =  - 0.6 *this.gamepad1.right_stick_x;

        
        if(this.gamepad1.right_trigger > 0.3)
        {
            speed_y = speed_y* 0.416666666;
            speed_x = speed_x* 0.416666666;
            speed_rotate = speed_rotate *0.416666666;
        }
      
        else if(this.gamepad1.right_bumper)
        {
            speed_y = speed_y * 0.25;
            speed_x = speed_x* 0.25;
            speed_rotate = speed_rotate * 0.25;
        }

        double angle = Math.toDegrees( Math.atan2(speed_y,speed_x));
        double speed = Math.sqrt(2) * Math.sqrt((speed_y*speed_y) + (speed_x *speed_x));

        double diagonal1 = -speed * Math.sin(Math.toRadians(angle+45));
        double diagonal2 = speed * Math.cos(Math.toRadians(angle+45));
        
        double gyroPower = 0;
        
        telemetry.addData("SpeedX", diagonal1);
        telemetry.addData("SpeedY", diagonal2);
        telemetry.addData("Speed rotate", speed_rotate);
        telemetry.update();
        
        
        driveTrain.setDiagonalPower(diagonal1,diagonal2, speed_rotate + gyroPower);
        if( this.gamepad1.touchpad || this.gamepad1.guide)
            imu.resetYaw();

    }

    double getPowerForVoltage(double voltage){
        return voltage/voltageSensor.getVoltage();
    }
    
    public boolean must(){
           return opModeIsActive();
    }


    protected void initManualBot() {
        List<String> motorConfigs = Arrays.asList("rightFront", "leftFront", "rightBack", "leftBack");
        List<DcMotorImplEx> driveMotors = motorConfigs.stream().map(config -> hardwareMap.get(DcMotorImplEx.class, config)).collect(Collectors.toList());
        driveTrain = new Drive(driveMotors);
        driveTrain.setMotorMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        datalog = new Datalog("datalog_01.csv");

        //init IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        // init voltage Sensor
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        //create controllers
        imuController = new IMUController(imu,driveTrain);
        // DistanceController distanceController = new DistanceController(driveTrain);
    }


     public static class Button {
        boolean lastButtonState;
        private final Supplier<Boolean> supplier;


        public Button(Supplier<Boolean> button) {
            supplier = button;
        }

        boolean onPressed(){
            Boolean currentState = supplier.get();
            //current state == true and lastButtonState == false that mean the button is just pressed
            boolean output = currentState && !lastButtonState;
            lastButtonState = currentState;
            return output;
        }

        boolean onReleased(){
            Boolean currentState = supplier.get();
            //current state == false and lastButtonState == true that mean the button is just released
            boolean output = !currentState && lastButtonState;
            lastButtonState = currentState;
            return output;
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
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField elbowPosition      = new Datalogger.GenericField("Elbow Position");
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
                            battery
                            
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

