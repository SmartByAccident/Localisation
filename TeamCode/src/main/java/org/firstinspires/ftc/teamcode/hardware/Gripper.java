package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;


public class Gripper {
    Servo servo;
    double open;
    double close;
    
    // todo: write your code here
    public Gripper(Servo servo, double open, double close){
        this.servo = servo;
        this.open = open;
        this.close = close;
    }
    
    public void open(){
        servo.setPosition(open);
    }
    public void close(){
        servo.setPosition(close);
    }
}
