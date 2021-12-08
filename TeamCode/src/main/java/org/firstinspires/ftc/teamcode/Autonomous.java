package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "FreightFenzy_REDAuton1 (Java)")
public class ProtoCompRobotMove extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive, intakeMotor, lifterMotor;
    private CRServo bucketServo;

    //For performance measuring
    private double prevElapsedTime = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        lifterMotor = hardwareMap.get(DcMotor.class, "lifter_motor");
        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

        bucketServo = hardwareMap.get(CRServo.class, "bucket_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);  //Motor wires are backwards, put direction to FORWARD when fixed
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        prevElapsedTime = 0;

        //Convert from the counts per revolution of the encoder to counts per inch
        static final double HD_COUNTS_PER_REV = 28;
        static final double DRIVE_GEAR_REDUCTION = 20.15293;
        static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
        static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
        static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;


        private void drive(double power, double leftBackInches, double leftFrontInches, double rightBackInches, double rightFrontInches) {
            int leftBackTarget;
            int leftFrontTarget;
            int rightBackTarget;
            int rightFrontTarget;

            if (opModeIsActive()) {
                // Create target positions
                leftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * DRIVE_COUNTS_PER_IN);
                leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * DRIVE_COUNTS_PER_IN);
                rightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * DRIVE_COUNTS_PER_IN);
                rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * DRIVE_COUNTS_PER_IN);

                // set target position
                leftBackDrive.setTargetPosition(leftBackTarget);
                rightBackDrive.setTargetPosition(rightBackTarget);
                leftFrontDrive.setTargetPosition(leftFrontTarget);
                rightFrontDrive.setTargetPosition(rightFrontTarget);


                //switch to run to position mode
                leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                //run to position at the desiginated power
                leftBackDrive.setPower(power);
                rightBackDrive.setPower(power);
                leftFrontDrive.setPower(power);
                rightFrontDrive.setPower(power);

                // wait until both motors are no longer busy running to position
                while (opModeIsActive() && (LeftDrive.isBusy() || RightDrive.isBusy())) {
                }

                // set motor power back to 0
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
            }
        }
}

