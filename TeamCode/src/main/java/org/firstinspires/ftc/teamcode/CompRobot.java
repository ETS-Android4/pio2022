package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

/*
* This class is used for constants that will be used for the robot.
* It is here to easily change behaviors of all programs
 */

public class CompRobot {
    //Variables for the heading PID loop, which prevents outside forces from rotating the robot
    public static double HeadingKp = 1, HeadingKi = 0.1, HeadingKd = 0.1;

    //Below a certain power, the motors will not move but make noise
    //This function adds a minimum power to prevent that
    public static double stallPower(double input, double minPower){
        if(Math.abs(input) < minPower) return 0;
        return input;
    }

    public DcMotor leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive, intakeMotor;

    public CompRobot(){
        this(
                hardwareMap.get(DcMotor.class, "left_back_drive"),
                hardwareMap.get(DcMotor.class, "left_front_drive"),
                hardwareMap.get(DcMotor.class, "right_back_drive"),
                hardwareMap.get(DcMotor.class, "right_front_drive"),
                hardwareMap.get(DcMotor.class, "intake_motor")
        );
    }

    public CompRobot(DcMotor leftBackDrive, DcMotor leftFrontDrive, DcMotor rightBackDrive, DcMotor rightFrontDrive, DcMotor intakeMotor){
        this.leftBackDrive = leftBackDrive;
        this.leftFrontDrive = leftFrontDrive;
        this.rightBackDrive = rightBackDrive;
        this.leftFrontDrive = rightFrontDrive;
        this.intakeMotor = intakeMotor;

    }

    public String move(double drive, double strafe, double turn){
        double leftFrontPower = CompRobot.stallPower(Range.clip(drive - turn + strafe, -1.0, 1.0), 0.1);
        double leftBackPower = CompRobot.stallPower(Range.clip(drive - turn - strafe, -1.0, 1.0),0.1);
        double rightFrontPower = CompRobot.stallPower(Range.clip(drive + turn - strafe, -1.0, 1.0),0.1);
        double rightBackPower = CompRobot.stallPower(Range.clip(drive + turn + strafe, -1.0, 1.0),0.1);

        // Send calculated power to wheels
        this.leftBackDrive.setPower(leftBackPower);
        this.leftFrontDrive.setPower(leftFrontPower);
        this.rightBackDrive.setPower(rightBackPower);
        this.rightFrontDrive.setPower(rightFrontPower);

        return "%.2f";
    }
}
