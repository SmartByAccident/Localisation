package org.firstinspires.ftc.teamcode.hardware;

public enum GripperServo {
    LEFT_GRIPPER(0.1,0.7), RIGHT_GRIPPER(0.6,0.15);

    public final double open;
    public final double closed;

    GripperServo(double open, double closed) {
        this.open = open;
        this.closed = closed;
    }

 
}
