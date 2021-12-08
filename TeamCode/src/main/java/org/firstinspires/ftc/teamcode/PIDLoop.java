package org.firstinspires.ftc.teamcode;

public class PIDLoop {
    public double kp, ki, kd, goal, outMin, outMax, minError = 0;
    protected double pTerm, iTerm, dTerm, prevError = 0, iPreTime = 0, dPreTime = 0, gPreTime = 0;

    //CONSTRUCTORS
    public PIDLoop(){
        this(0, 0, 0, 0, -1, 1);
    }
    public PIDLoop(double kp, double ki, double kd){
        this(kp, ki, kd, 0, -1, 1);
    }
    public PIDLoop(double kp, double ki, double kd, double goal){
        this(kp, ki, kd, goal, -1, 1);
    }
    public PIDLoop(double kp, double ki, double kd, double goal, double outMin, double outMax){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.goal = goal;
        this.outMin = outMin;
        this.outMax = outMax;
    }

    public void reset(){
        iTerm = 0;
    }

    public void reset(double time){
        reset();
        iPreTime = time;
    }

    //Use this to calculate output using input and time
    public double update(double input, double time){
        pTerm = calculateP(input);
        iTerm = calculateI(input, time);
        dTerm = calculateD(input, time);
        return this.output();
    }

    public double error(double input){
        return goal - input;
    }

    public double calculateP(double input){
        if(Math.abs(error(input)) < minError){
            return 0;
        }
        return error(input);
    }

    public double calculateI(double input, double time){
        double tempI;
        if(ki ==0)return 0;
        if(error(input) * kp > outMax || error(input) * kp < outMin){
            tempI = 0;
        } else {
            tempI = Math.min(outMax, Math.max(outMin, ki * (iTerm + error(input) * (time - iPreTime)))) / ki;
        }
        iPreTime = time;
        return tempI;
    }

    public double calculateD(double input, double time){
        double tempD = dTerm;
        if(time != dPreTime) {
            tempD = ((error(input)) - prevError) / (time - dPreTime);
        }
        prevError = error(input);
        dPreTime = time;
        return tempD;
    }

    //Use this to get previous results without putting an input
    public double output(){
        return Math.min(outMax, Math.max(outMin, kp * pTerm + ki * iTerm + kd * dTerm));
    }

    //Method to move the goal by a certain amount
    //Uses include controlling goal with a joystick on a gamepad
    public void moveGoal(double goalChange, double time){
        goal += goalChange * (time - gPreTime);
        gPreTime = time;
    }

    public double p(){
        return pTerm;
    }
    public double i(){
        return iTerm;
    }
    public double d(){
        return dTerm;
    }
}
