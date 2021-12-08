package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
* This class is used for constants that will be used for the robot.
* It is here to easily change behaviors of all programs
 */

public class CompRobot {
    //Variables for the heading PID loop, which prevents outside forces from rotating the robot
    public static double HeadingKp = 1, HeadingKi = 0.1, HeadingKd = 0.1;
    public static double LifterKp = 0.005, LifterKi = 0.0001, LifterKd = 0.0001;
    private PIDLoop lifter;

    //Below a certain power, the motors will not move but make noise
    //This function adds a minimum power to prevent that
    public static double stallPower(double input, double minPower){
        if(Math.abs(input) < minPower) return 0;
        return input;
    }

    public DcMotor leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive, intakeMotor;
    public DcMotor lifterMotor;
    public CRServo bucketServo;

    public void init(HardwareMap components){

        leftBackDrive = components.get(DcMotor.class, "left_back_drive");
        leftFrontDrive = components.get(DcMotor.class, "left_front_drive");
        rightBackDrive = components.get(DcMotor.class, "right_back_drive");
        rightFrontDrive = components.get(DcMotor.class, "right_front_drive");
        intakeMotor = components.get(DcMotor.class, "intake_motor");
        lifterMotor = components.get(DcMotor.class, "lifter_motor");
        bucketServo = components.get(CRServo.class, "bucket_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);  //Motor wires are backwards, put direction to FORWARD when fixed
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lifter = new PIDLoop(LifterKp, LifterKi, LifterKd, 0, -1, 1);
        lifter.minError=15;

        lifterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //lifterMotor.setPower(0.5);
        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        lifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

    public String lifter(boolean up, boolean down, boolean forward, boolean back, double time){
        if(up){
            lifter.kp = LifterKp;
            lifter.ki = LifterKi;
            lifter.goal = 2600;
            //

        }
        else if(down){
            lifter.kp = 0.05*LifterKp;
            lifter.ki = 0;
            lifter.goal = 0;
        }


        lifterMotor.setPower(stallPower(-lifter.update(lifterMotor.getCurrentPosition(), time),0.05));

        if(forward)bucketServo.setPower(1);
        else if(back)bucketServo.setPower(-1);
        else bucketServo.setPower(0);

        return "LIFTER MOTOR| POS: " + lifterMotor.getCurrentPosition() + " POW: " + lifterMotor.getPower() + " ACTIVE: " + lifterMotor.isBusy() +
                "\nP: " + lifter.p() + " I: " + lifter.i() + " D: " + lifter.d() + " OUTPUT: " + lifter.output();
    }
}
