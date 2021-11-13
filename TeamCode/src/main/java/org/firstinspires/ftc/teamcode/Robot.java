package org.firstinspires.ftc.teamcode;

/*
* This class is used for constants that will be used for the robot.
* It is here to easily change behaviors of all programs
 */

public class Robot {
    //Variables for the heading PID loop, which prevents outside forces from rotating the robot
    public static double HeadingKp = 1, HeadingKi = 0.1, HeadingKd = 0.1;

    //Below a certain power, the motors will not move but make noise
    //This function adds a minimum power to prevent that
    public static double stallPower(double input, double minPower){
        if(Math.abs(input) < minPower) return 0;
        return input;
    }
}
