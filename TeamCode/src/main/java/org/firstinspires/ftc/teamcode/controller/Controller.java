package org.firstinspires.ftc.teamcode.controller;

/**
 * This is an template for controller used on the robot.
 */
public abstract class Controller<T>{
    public T target;

    /**
     * This method manipulates the respective accutators to reach the current target value.
     * Note: do not use any loops inside this method.
     * @return true: when the target value is reached.
     */
    public abstract boolean run();

    /**
     * @return the curresnt sensor value.
     */
    public abstract T getCurrentSensorValue();

    /**
     * Set target value for the controller.
     */
    public void setTarget(T target){
        this.target=target;
    }

    public double maximumSecondsRequiredToReachTarget(){
        return 4;
    }

    
    
    
}

