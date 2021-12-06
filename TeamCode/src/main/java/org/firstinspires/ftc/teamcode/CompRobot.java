package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public DcMotor leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive, intakeMotor, lifterMotor;
    public CRServo bucketServo;

    public void init(HardwareMap components){

        leftBackDrive = components.get(DcMotor.class, "left_back_drive");
        leftFrontDrive = components.get(DcMotor.class, "left_front_drive");
        rightBackDrive = components.get(DcMotor.class, "right_back_drive");
        rightFrontDrive = components.get(DcMotor.class, "right_front_drive");
        intakeMotor = components.get(DcMotor.class, "intake_motor");
        lifterMotor = components.get(DcMotor.class, "lifter_motor");
        bucketServo = components.get(CRServo.class, "bucket_servo");

    }

    public String move(double drive, double strafe, double turn){
        drive = -drive;
        strafe = -strafe;//We switch the direction of the robot from intake front to intake back
        double leftFrontPower = CompRobot.stallPower(Range.clip(drive - turn + strafe, -1.0, 1.0), 0.1);
        double leftBackPower = CompRobot.stallPower(Range.clip(drive - turn - strafe, -1.0, 1.0),0.1);
        double rightFrontPower = CompRobot.stallPower(Range.clip(drive + turn - strafe, -1.0, 1.0),0.1);
        double rightBackPower = CompRobot.stallPower(Range.clip(drive + turn + strafe, -1.0, 1.0),0.1);

        // Send calculated power to wheels
        this.leftBackDrive.setPower(leftBackPower);
        this.leftFrontDrive.setPower(leftFrontPower);
        this.rightBackDrive.setPower(rightBackPower);
        this.rightFrontDrive.setPower(rightFrontPower);

        return String.format("LF: %.2f RF: %.2f LB: %.2f RB: %.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public String intake(boolean forward, boolean backward){
        //Intake motor control
        if(backward){
            this.intakeMotor.setPower(-1);
            return "IM: FULL OUT";
        } else if(forward){
            this.intakeMotor.setPower(1);
            return "IM: FULL IN";
        } else {
            this.intakeMotor.setPower(0);
            return "IM: STBY";
        }
    }

    public String lifter(boolean up, boolean down, boolean forward, boolean back){
        if(up)lifterMotor.setPower(1);
        else if(down)lifterMotor.setPower(-1);
        else lifterMotor.setPower(0);

        if(forward)bucketServo.setPower(1);
        else if(back)bucketServo.setPower(-1);
        else bucketServo.setPower(0);

        return "All good";
    }
}
