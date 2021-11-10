package org.firstinspires.ftc.teamcode;

//This PID loop is only to be used with RADIANS
//This PID loop accounts for the fact that moving past PI will go to -PI
//This PID loop may not work as intended if refresh rate is low
public class HdgPID extends PIDLoop{
    private int state = 0;
    private double prevD = 1;

    public HdgPID(){
        this(0, 0, 0, 0, -1, 1);
    }
    public HdgPID(double kp, double ki, double kd){
        this(kp, ki, kd, 0, -1, 1);
    }
    public HdgPID(double kp, double ki, double kd, double goal){
        this(kp, ki, kd, goal, -1, 1);
    }
    public HdgPID(double kp, double ki, double kd, double goal, double outMin, double outMax){

        super(kp, ki, kd, goal, outMin, outMax);
    }

    public int getState(){
        return state;
    }

    public double update(double input, double time){
        pTerm = calculateP(input, time);
        iTerm = calculateI(time);
        dTerm = calculateD(input, time);
        return output();
    }

    public double calculateP(double input, double time){
        if(Math.abs(goal-input) < Math.min(Math.abs(goal + 2 * Math.PI - input), Math.abs(goal - 2 * Math.PI - input))){
            return goal-input;
        }else if(Math.abs(goal + 2 * Math.PI -input) < Math.abs(goal - 2 * Math.PI - input)){
            return goal + 2 * Math.PI - input;
        }else{
            return goal - 2 * Math.PI - input;
        }
    }

    public double calculateD(double input, double time){

            if (Math.abs((input - preInput) / (time - dPreTime)) < Math.min(Math.abs((input + 2 * Math.PI - preInput) / (time - dPreTime)), Math.abs((input - 2 * Math.PI - preInput) / (time - dPreTime)))) {
                return super.calculateD(input,time);

            }else if(Math.abs((input + 2 * Math.PI - preInput) / (time - dPreTime)) < Math.abs((input - 2 * Math.PI - preInput) / (time - dPreTime))){
                return super.calculateD(input + 2 * Math.PI, time);

            }else{
                return super.calculateD(input - 2 * Math.PI, time);

            }


    }

    //Method to move the goal by a certain amount
    //Uses include controlling heading goal with a joystick on a gamepad
    public void moveGoal(double goalChange, double time, double input){
        goal = Math.max(input-Math.PI,Math.min(input+Math.PI,goal + goalChange * (time - gPreTime)));
        if(goal < -Math.PI){
            goal += 2 * Math.PI;
        }else if(goal > Math.PI){
            goal -= 2 * Math.PI;
        }
        gPreTime = time;
        this.reset();
    }

    public double manage(double sensorInput, double userInput, double time){
        if(Math.abs(userInput) >= 0.1) state = 2;
        switch(state) {
            case 0:
                return this.update(sensorInput, time);
            case 1:
                double tempD = this.calculateD(sensorInput, time);
                if(tempD/prevD < 0){
                    this.goal = sensorInput;
                    state--;
                    return 0;
                }
                prevD = tempD;
                return tempD * kd * 3;
            case 2:
                this.goal = sensorInput;
                state--;
                prevD = this.calculateD(sensorInput, time);
                return -userInput;
        }
        /*
        if(Math.abs(userInput) <= 0.1){
            return this.update(sensorInput, time);
        }else{
            this.goal = sensorInput;
            return userInput;
        }

         */
        return 0;
    }
}
