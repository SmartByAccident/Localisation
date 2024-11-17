package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.utilities.PIDCalculator;
import org.firstinspires.ftc.teamcode.utilities.FieldColor;
public class ForeArmController extends Controller<Integer> {
    private final DcMotor arm;
    private final DcMotor encoder;
    public final PIDCalculator pidCalculator= new PIDCalculator();
    private double power = 0;

    VoltageSensor voltageSensor;
    public ForeArmController(DcMotor encoder,DcMotor arm,VoltageSensor voltageSensor) {
        this.arm = arm;
        this.encoder = encoder;
        this.voltageSensor = voltageSensor;
        pidCalculator.set(0.0075,0.0005,0.0085, 5.5, 1);
        target = 0;
    }

    public double getRotationPower(){
        return 0;
        // if(power != 0){
        //   return power;
        // }
        // double currentAngle = getCurrentSensorValue();
        // pidCalculator.compute(currentAngle, target);
        
        // double pidOutput = pidCalculator.output;
        
        // return getPowerForVoltage(pidOutput);
    }
    
    public void setAngleTarget(double angle){
        int encoderTicks =(int) Math.floor(angle*22.777);
        super.setTarget(encoderTicks);
    }
    @Override
    public boolean run(){
        double currentAngle = getCurrentSensorValue();
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(getRotationPower());
        return reached();
    }
    
    public boolean reached(){
        return Math.abs(getCurrentSensorValue()-target) < 45;
    }
    
    @Override
    public Integer getCurrentSensorValue() {
        return encoder.getCurrentPosition();
    }
    double getPowerForVoltage(double voltage){
        return voltage/ voltageSensor.getVoltage();
    }
    public void setAdditionalInformation(double power){
        this.power =power;
    }
    public String toString(){
        String data = "";
        data += getCurrentSensorValue() + ",";
        data += getRotationPower() + ",";
        data += pidCalculator.output + ",";
        return data;
    }
}
