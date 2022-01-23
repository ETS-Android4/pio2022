package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/*
* This class is used for constants that will be used for the robot.
* It is here to easily change behaviors of all programs
 */

public class CompRobot {
    private static final String VUFORIA_KEY =
            "AVdeq0n/////AAABmVubfnY43U00u/7C9jL7D7oEhvoAVlRtTnRA0jLBMU5XVhldSJ69NqUCXighmROl5CFKotznVZCqEwgbVZHaCjGDE8WQOeaNsYvRwx1qOon8A707S2Pq6HkDu+zP5R0RUfyL/SgchAbEYTwmhMlk09JmA21ex+vQKrz4ICh7Pu6F4+M304VCrk38wIJpnN6Oa9ElxvqXfJkt8Nv4gLeCJbIoawnR/b7x8VSMGXjPdnMPcCQDpNCFMnXR5PA75q1oMeAXgmjQ/302AD6SqOIop0bzdOSr2tH+mBt+HPPGO8clJ/TS7sCV8MSMTSTFPlxn/yF44XSUCu6n1+HJaMOwW7OnYVjmFJlmlJbKn7tM+KKS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};

    public static double HeadingKp = 10, HeadingKi = 0.1, HeadingKd = 0.1;//Variables for the heading PID loop, which can control the robot's heading
    public static double LifterKp = 0.005, LifterKi = 0.001, LifterKd = 0.0001;//Variables for the lifter PID loop, which controls how the lifter goes to it's target
    public static int[] levels = {0,1800,2600};//Each number represents different levels from down to up

    //Below a certain power, the motors will not move but make noise
    //This function adds a minimum power to prevent that
    public static double stallPower(double input, double minPower){
        if(Math.abs(input) < minPower) return 0;
        return input;
    }

    public ElapsedTime eTime  = new ElapsedTime();
    private boolean preToggle = false, secFloor = false, lifterUp = false, drop = false;

    public DcMotor leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive, intakeMotor,
            lifterMotor, carouselMotor;
    public CRServo bucketServo;
    public BNO055IMU imu;
    public Orientation angles;
    public HdgPID direction = new HdgPID(HeadingKp, HeadingKi, HeadingKd);
    public PIDLoop lifterPID;


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
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);  //Motor wires are backwards, put direction to FORWARD when fixed
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

        lifterPID = new PIDLoop(LifterKp, LifterKi, LifterKd, 0, -1, 1);
        lifterPID.minError=15;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
    }

    public void initVuforia(HardwareMap components) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = components.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        int tfodMonitorViewId = components.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", components.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1 , 16.0/9.0);
        }
    }

    public List<Recognition> runTFod(){
        return tfod.getUpdatedRecognitions();
    }

    public String move(double drive, double strafe, double turn){
        drive = -drive;
        strafe = -strafe;//We switched the direction of the robot from intake front to intake back
        turn = -turn;
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
            intakeMotor.setPower(-0.60);
        } else if(forward){
            intakeMotor.setPower(0.60);
        } else {
            intakeMotor.setPower(0);
        }
        return String.format("IM: %.2f", intakeMotor.getPower());
    }

    public String lifter(boolean up, boolean down, boolean forward, boolean back, boolean toggle){

        if(toggle && !preToggle){
            if(!lifterUp) {
                lifterPID.kp = LifterKp;
                lifterPID.ki = LifterKi;
                lifterUp = true;
                if (secFloor) lifterPID.goal = levels[1];
                else lifterPID.goal = levels[2];
            }else{
                lifterUp = false;
                lifterPID.goal = levels[0];
                lifterPID.kp = 2 * LifterKp;
                lifterPID.ki = 0 * LifterKi;

            }

        }

        if(up){
            secFloor = false;
            if(lifterUp) lifterPID.goal = levels[2];
        }else if(down){
            secFloor = true;
            if(lifterUp) lifterPID.goal = levels[1];
        }

        lifterMotor.setPower(stallPower(-lifterPID.update(lifterMotor.getCurrentPosition(), eTime.time()),0.05));

        if(forward && !drop)bucketServo.setPower(1);
        else if(back && !drop)bucketServo.setPower(-1);
        else if(!drop) bucketServo.setPower(0);

        preToggle = toggle;
        return "LIFTER MOTOR| POS: " + lifterMotor.getCurrentPosition() + " POW: " + lifterMotor.getPower() +
                "\n\t" + String.format("PID : (%.2f, %.2f, %.2f)", lifterPID.p(), lifterPID.i(), lifterPID.d()) + " OUTPUT: " + lifterPID.output() +
                "\n\tSECOND FLOOR: " + secFloor;
    }

    public String carousel(boolean forward, boolean backward){
        if(forward)carouselMotor.setPower(0.25);
        else if (backward) carouselMotor.setPower(-0.25);
        else carouselMotor.setPower(0);
        return String.format("CM: %.2f", carouselMotor.getPower());
    }


    public String getAngles(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        return String.format("(%.2f, %.2f, %.2f)", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
    }

    public double currentDirection(){
        getAngles();
        return angles.secondAngle;
    }

    public double directionError(){
        return direction.error(currentDirection());
    }
}
