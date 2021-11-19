/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file tests the proto competition robot's movement
 *
 * This particular OpMode just executes movement for a four wheeled robot.
 * This opmode prevents turning when the driver doesn't command a turn e.g. uneven motor
 */

@TeleOp(name="Assisted Movement", group="Proto Comp Robot")

public class ProtoCompRobotMoveA extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive, intakeMotor;
    BNO055IMU imu;

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);  //Motor wires are backwards, put direction to FORWARD when fixed
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        //IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //IMU initialization
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //PID controller to avoid unwanted rotation
        HdgPID hdgHold = new HdgPID(CompRobot.HeadingKp, CompRobot.HeadingKi, CompRobot.HeadingKd);
        hdgHold.minError = 0.1;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        prevElapsedTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Read orientation data from the IMU
            Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            // This mode uses left stick to translate, and right stick to rotate.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y; //Move left stick up/down to move forward/backward
            double strafe = gamepad1.left_stick_x; //Move left stick right/left to move right/left
            double turn = -hdgHold.manage(-angles.firstAngle, -gamepad1.right_stick_x, getRuntime()); //Move right stick right/left to turn right/left. Adjust if robot is turning without driver input

            double leftFrontPower = CompRobot.stallPower(Range.clip(drive - turn + strafe, -1.0, 1.0), 0.1);
            double leftBackPower = CompRobot.stallPower(Range.clip(drive - turn - strafe, -1.0, 1.0),0.1);
            double rightFrontPower = CompRobot.stallPower(Range.clip(drive + turn - strafe, -1.0, 1.0),0.1);
            double rightBackPower = CompRobot.stallPower(Range.clip(drive + turn + strafe, -1.0, 1.0),0.1);

            // Send calculated power to wheels
            leftBackDrive.setPower(leftBackPower);
            leftFrontDrive.setPower(leftFrontPower);
            rightBackDrive.setPower(rightBackPower);
            rightFrontDrive.setPower(rightFrontPower);

            //Intake motor control
            if(gamepad1.left_bumper)  intakeMotor.setPower(-1);
            else if(gamepad1.right_bumper) intakeMotor.setPower(1);
            else intakeMotor.setPower(0);

            // Show the elapsed game time, performance, and wheel power.
            telemetry.addData("Status", "\n\tRun Time: %.2f\n\tTPS: %.2f", getRuntime(), 1/(getRuntime()-prevElapsedTime));
            telemetry.addData("Motors", "\n\tLF(%.2f)\tRF(%.2f)\n\tLB(%.2f)\tRB(%.2f)",
                    leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Heading: ", "%.2f°, Goal: %.2f°, Error: %.2f°", -180*angles.firstAngle/Math.PI, -180*hdgHold.goal/Math.PI, -180*hdgHold.error(angles.firstAngle)/Math.PI); //Show current heading, goal, and error of robot
            telemetry.addData("PID status:", "p: %.2f\ti: %.2f\td: %.2f\tstate:%d", hdgHold.p(), hdgHold.i(), hdgHold.d(),hdgHold.getState()); //Show information from heading PID loop
            telemetry.addData("Additional PID info: ", "input: %.2f, prevError: %.2f, preTime: %.2f", angles.firstAngle, hdgHold.prevError, hdgHold.dPreTime); //Even more information
            telemetry.update();
            prevElapsedTime = getRuntime();
        }
    }
}
