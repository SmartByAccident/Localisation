package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDCalculator {
    private double kP,kI,kD; //control constants for PID algorithm.
    /**
     * minimumTimeBetweenTwoCalcuations defines the minimum time between two PID calcuations.
     * This is done so that the PID integration is not done on every execution of compute method.
     * This allows the accutors to react to the last input value.
     */
    private final double minimumTimeBetweenTwoCalculations;
    private double lastTimeWhenPIEDWasCalculated;
    private static final ElapsedTime clock = new ElapsedTime(); // to maintain time

    public double integralSum;
    private double outMax;
    private double outMin;
    private double lastInput;
    public double output;

    public PIDCalculator()
    {
        kP=0.07;
        kI=0.01;
        kD=0.17;
        // kP=0.01;
        // kI=0.0;
        // kD=0.15;
        outMax=0.8;
        outMin=0.1;
        integralSum = 0;
        output=0;
        lastInput = 0;
        lastTimeWhenPIEDWasCalculated =0;
        minimumTimeBetweenTwoCalculations =0.004;
        
        // minimumTimeBetweenTwoCalculations =1;

    }

    /**
     * This method clears the integral erro calcuted uptill this point.
     * This is used in case where the integrall error accumated is too large.
     */
    public void clear_error()
    {
        integralSum =0;
    }

    /**
     * This method calculated the PIDController value for the given input sensor value and target position.
     * @return true if the new PID values are calculated,
     */
    public boolean compute(double input,double setpoint)
    {
        double timeNow = clock.seconds();
        double timeSinceLastCalculation = (timeNow - lastTimeWhenPIEDWasCalculated);

        if(timeSinceLastCalculation < minimumTimeBetweenTwoCalculations){
            return false;
        }


        /*Compute the error value*/

        double error = setpoint - input;
        double dInput = integralSum == 0 ? 0 : (input - lastInput);
        integralSum += (kI * error); //add integral sum


        /*Add 
        on Measurement*/
        // integralSum-= kP * dInput; //reduduce the integral value, this is done to prevent the error accumlation.

        if(integralSum >0.75* outMax)
            integralSum= 0.75*outMax;
        else if(integralSum < outMin)
            integralSum= outMin;

        /*Add Proportional on Error*/

        output = kP * error;


        /*Compute Rest of PID Output*/
        output += integralSum - kD * dInput;

        double absoluteOutput=Math.abs(output);
        if(absoluteOutput > outMax)
            absoluteOutput = outMax;
        else if(absoluteOutput < outMin)
            absoluteOutput = outMin;
            
        if(output<0)
            absoluteOutput*=-1;
        output=absoluteOutput;


        /*Remember some variables for next time*/
        lastInput = input;
        lastTimeWhenPIEDWasCalculated = timeNow;
        return true;

    }


    public void set(double kP,double kI ,double kD, double outMax, double outMin)

    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.outMax = outMax;
        this.outMin = outMin;
    }



}

