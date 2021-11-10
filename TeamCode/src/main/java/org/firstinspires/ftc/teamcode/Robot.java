package org.firstinspires.ftc.teamcode;

public class Robot {
    public static double kp = 4;
    public static double ki = 6;
    public static double kd = 0.1;

    public static double stallPower(double input, double minPower){
        if(Math.abs(input) < minPower) return 0;
        return input;
    }
}
