package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
* This class is used for constants that will be used for the robot.
* It is here to easily change behaviors of all programs
 */

public class CompRobot {
    public static double HeadingKp = 1, HeadingKi = 0.1, HeadingKd = 0.1;//Variables for the heading PID loop, which can control the robot's heading
    public static double LifterKp = 0.005, LifterKi = 0.001, LifterKd = 0.0001;//Variables for the lifter PID loop, which controls how the lifter goes to it's target
    public static int[] levels = {0,1800,3100};//Each number represents different levels from down to up

    //Below a certain power, the motors will not move but make noise
    //This function adds a minimum power to prevent that
    public static double stallPower(double input, double minPower){
        if(Math.abs(input) < minPower) return 0;
        return input;
    }

    private ElapsedTime eTime  = new ElapsedTime();
    private PIDLoop lifter;
    private boolean preFloorSwitch = false, secFloor = false, lifterUp = false, drop = false;
    private double rServoStart;
    private double[] rServoDuration = {0, 0.8};
    private int rServoPhase = 0;

    public DcMotor leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive, intakeMotor,
            lifterMotor, carouselMotor;
    public CRServo bucketServo;
    public BNO055IMU imu;


    public void init(HardwareMap components){

        leftBackDrive = components.get(DcMotor.class, "left_back_drive");
        leftFrontDrive = components.get(DcMotor.class, "left_front_drive");
        rightBackDrive = components.get(DcMotor.class, "right_back_drive");
        rightFrontDrive = components.get(DcMotor.class, "right_front_drive");
        intakeMotor = components.get(DcMotor.class, "intake_motor");
        lifterMotor = components.get(DcMotor.class, "lifter_motor");
        carouselMotor = components.get(DcMotor.class, "carousel_motor");

        bucketServo = components.get(CRServo.class, "bucket_servo");

        imu = components.get(BNO055IMU.class, "imu");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);  //Motor wires are backwards, put direction to FORWARD when fixed
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        lifterMotor.setDirection(DcMotor.Direction.FORWARD);
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);

        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter = new PIDLoop(LifterKp, LifterKi, LifterKd, 0, -1, 1);
        lifter.minError=15;


        //lifterMotor.setPower(0.5);





        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);


    }

    public String move(double drive, double strafe, double turn){
        drive = -drive;
        strafe = -strafe;//We switched the direction of the robot from intake front to intake back
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
            intakeMotor.setPower(-0.45);
        } else if(forward){
            intakeMotor.setPower(0.45);
        } else {
            intakeMotor.setPower(0);
        }
        return String.format("IM: %.2f", intakeMotor.getPower());
    }

    public String lifter(boolean up, boolean down, boolean forward, boolean back, boolean floorSwitch) throws Exception {

        if((up || !floorSwitch && preFloorSwitch && lifterUp) && !drop){
            lifter.kp = LifterKp;
            lifter.ki = LifterKi;
            lifterUp = true;
            if(secFloor)lifter.goal = levels[1];
            else lifter.goal = levels[2];

        } else if(down && lifterUp){
            drop = true;
            rServoPhase = 1;
            rServoStart = eTime.time();
            lifter.kp = 2*LifterKp;
            lifter.ki = 0*LifterKi;
        }
        switch(rServoPhase){
            case 0:
                break;
            case 1:
                bucketServo.setPower(-1);
                if(rServoStart + rServoDuration[0] > eTime.time()){
                    break;
                }
                lifter.goal = levels[1];
                rServoPhase = 2;
            case 2:
                bucketServo.setPower(0);
                if(Math.abs(lifter.error(lifterMotor.getCurrentPosition())) > lifter.minError) {

                    break;}
                rServoPhase = 3;
                rServoStart = eTime.time();
            case 3:
                bucketServo.setPower(-0.3);
                if(rServoStart + rServoDuration[1] > eTime.time()){
                    break;
                }
                bucketServo.setPower(0);
                lifter.goal = levels[0];
                rServoPhase = 0;
                drop = false;
                lifterUp = false;
                break;
            default:
                throw new Exception("How");
        }

        if(floorSwitch && !preFloorSwitch){
            secFloor = !secFloor;
        }


        lifterMotor.setPower(stallPower(-lifter.update(lifterMotor.getCurrentPosition(), eTime.time()),0.05));

        if(forward && !drop)bucketServo.setPower(1);
        else if(back && !drop)bucketServo.setPower(-1);
        else if(!drop) bucketServo.setPower(0);

        preFloorSwitch = floorSwitch;

        return "LIFTER MOTOR| POS: " + lifterMotor.getCurrentPosition() + " POW: " + lifterMotor.getPower() +
                "\n\t" + String.format("PID : (%.2f, %.2f, %.2f)", lifter.p(), lifter.i(), lifter.d()) + " OUTPUT: " + lifter.output() +
                "\n\tSECOND FLOOR: " + secFloor + " SERVO PHASE: " + rServoPhase;
    }

    public String carousel(boolean on){
        if(on)carouselMotor.setPower(0.5);
        else carouselMotor.setPower(0);
        return String.format("CM: %.2f", carouselMotor.getPower());
    }

    public String getAngles(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return String.format("(%.2f, %.2f, %.2f)", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
    }
}
