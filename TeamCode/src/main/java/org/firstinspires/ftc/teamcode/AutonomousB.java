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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpM
 * ode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Blue Carousel", group="Comp Robot")

public class AutonomousB extends LinearOpMode {

        /* Declare OpMode members. */
        CompRobot robot = new CompRobot();   // Use the common comp robot's hardware
        private ElapsedTime runtime = new ElapsedTime();
        double[][] duckPositions = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
        int level = 0, consDetections = 0;

        static final double FORWARD_SPEED = 0.3;
        static final double TURN_SPEED = 0.2;
        static final double STRAFE_SPEED = 0.6;
        static final double CAROUSEL_SPEED = 0.20;
        private DistanceSensor frontRange, rightRange, leftRange;


        @Override
        public void runOpMode() {

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.initDrive(hardwareMap);
            robot.initVuforia(hardwareMap);

            //you can use this as a regular distance sensor
            frontRange = hardwareMap.get(DistanceSensor.class, "front_distance");
            rightRange = hardwareMap.get(DistanceSensor.class, "right_distance");
            leftRange = hardwareMap.get(DistanceSensor.class, "left_distance");

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");//
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            //Start near warehouse
            while (opModeIsActive()) {
                sleep(500);
                //Duck Detection
                while (consDetections < 5) {
                    boolean success = false;
                    List<Recognition> objects = robot.runTFod();
                    sleep(100);
                    if (objects != null) {
                        for (Recognition recognition : objects) {
                            if (recognition.getLabel().equals("Duck")) {
                                success = true;
                                if (recognition.getRight() > 250) {
                                    if (level != 3) consDetections = 0;
                                    else consDetections++;
                                    level = 3;
                                } else {
                                    if (level != 2) consDetections = 0;
                                    else consDetections++;
                                    level = 2;
                                }
                            }
                        }
                    }
                    if (!success) {
                        if (level != 1) consDetections = 0;
                        else consDetections++;
                        level = 1;
                    }
                }
                telemetry.addData("Level", level);
                telemetry.update();

                //Move until 75cm away from wall
                while (frontRange.getDistance(DistanceUnit.CM) < 75) {
                    telemetry.addData("Level", level);
                    telemetry.addData("Distance", frontRange.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    robot.move(-FORWARD_SPEED, 0, 0);
                }

                runtime.reset();
                robot.direction.reset();
                robot.direction.goal = -1.5;
                robot.direction.minError = 0.1;
                while (Math.abs(robot.direction.error(robot.currentDirection())) > robot.direction.minError) {
                    telemetry.addData("Level", level);
                    telemetry.update();
                    robot.move(0, 0, robot.direction.update(robot.currentDirection(), runtime.seconds()));
                }
                telemetry.addData("Level", level);
                telemetry.addData("Lift", "Ready");
                telemetry.update();
                robot.move(0, 0, 0);
                robot.lifterPID.minError = 50;
                robot.dropperPID.minError = 30;
                if (level == 1) {

                }
                else if (level == 2) {

                }
                else if (level == 3) {
                    robot.lifterPID.goal = CompRobot.levels[3];
                    runtime.reset();
                    while (Math.abs(robot.lifterPID.error(robot.lifterMotor.getCurrentPosition())) > robot.lifterPID.minError) {
                        robot.lifterMotor.setPower(-robot.lifterPID.update(robot.lifterMotor.getCurrentPosition(), runtime.time()));
                        telemetry.addData("Level", level);
                        telemetry.update();
                    }
                    runtime.reset();
                    while (runtime.time() < 1.1) {
                        robot.bucketServo.setPower(1);
                    }
                    robot.bucketServo.setPower(0.05);
                }
                else {
                    telemetry.addData("fuck", "shit");
                    stop();
                }
                robot.lifterPID.goal = CompRobot.levels[1];
                runtime.reset();
                while (Math.abs(robot.lifterPID.error(robot.lifterMotor.getCurrentPosition())) > robot.lifterPID.minError) {
                    robot.lifterMotor.setPower(-robot.lifterPID.update(robot.lifterMotor.getCurrentPosition(), runtime.time()));
                    robot.dropperPID.kp = CompRobot.dropperKp;
                    robot.dropperPID.ki = CompRobot.dropperKi;
                    robot.dropperPID.kd = CompRobot.dropperKd;
                    robot.dropperMotor.setPower(robot.lifterPID.update(robot.lifterMotor.getCurrentPosition(), runtime.time()));
                    telemetry.addData("Level", level);
                    telemetry.update();
                }
                robot.lifterPID.goal = CompRobot.levels[0];
                runtime.reset();
                while (runtime.time() < 0.8) {
                    robot.bucketServo.setPower(-1);
                }
                robot.bucketServo.setPower(0);
                runtime.reset();
                while (Math.abs(robot.lifterPID.error(robot.lifterMotor.getCurrentPosition())) > robot.lifterPID.minError) {
                    robot.lifterMotor.setPower(-robot.lifterPID.update(robot.lifterMotor.getCurrentPosition(), runtime.time()));
                    robot.dropperPID.kp = CompRobot.dropperKp;
                    robot.dropperPID.ki = CompRobot.dropperKi;
                    robot.dropperPID.kd = CompRobot.dropperKd;
                    robot.dropperMotor.setPower(robot.lifterPID.update(robot.lifterMotor.getCurrentPosition(), runtime.time()));
                    telemetry.addData("Level", level);
                    telemetry.update();
                }
                robot.dropperMotor.setPower(0);
                robot.lifterMotor.setPower(0);
                runtime.reset();
                robot.direction.reset();
                robot.direction.goal = Math.PI / 2;
                robot.direction.minError = 0.1;
                while (Math.abs(robot.direction.error(robot.currentDirection())) > robot.direction.minError) {
                    telemetry.addData("Level", level);
                    telemetry.update();
                    robot.move(0, 0, robot.direction.update(robot.currentDirection(), runtime.seconds()));
                }
                while (frontRange.getDistance(DistanceUnit.CM) > 12) {
                    telemetry.addData("Distance", frontRange.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    robot.move(FORWARD_SPEED, 0, 0);
                }
                while (rightRange.getDistance(DistanceUnit.CM) > 30) {
                    telemetry.addData("Distance", rightRange.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    robot.move(0, STRAFE_SPEED, 0);
                }
                runtime.reset();
                while (runtime.time() < 0.4) {
                    telemetry.addData("Finalizing Distance", "All good");
                    telemetry.update();
                    robot.move(0, STRAFE_SPEED * 0.4, 0);
                }
                robot.move(0, 0, 0);

                runtime.reset();
                while (runtime.time() < 2) {
                    telemetry.addData("Spinning", "All good");
                    robot.carouselMotor.setPower(CAROUSEL_SPEED);
                }
                runtime.reset();
                while (runtime.time() < 0.4) {
                    telemetry.addData("Going away from carousel", "All good");
                    telemetry.update();
                    robot.move(0, -STRAFE_SPEED, 0);
                }
                runtime.reset();
                robot.direction.reset();
                robot.direction.goal = 0;
                robot.direction.minError = 0.1;
                while (Math.abs(robot.direction.error(robot.currentDirection())) > robot.direction.minError) {
                    telemetry.addData("Level", level);
                    telemetry.update();
                    robot.move(0, 0, robot.direction.update(robot.currentDirection(), runtime.seconds()));
                }
                while (frontRange.getDistance(DistanceUnit.CM) < 65) {
                    telemetry.addData("Level", level);
                    telemetry.addData("Distance", frontRange.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    robot.move(FORWARD_SPEED, 0, 0);
                }
                while (rightRange.getDistance(DistanceUnit.CM) > 20) {
                    telemetry.addData("Level", level);
                    telemetry.addData("Distance", frontRange.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    robot.move(0, -STRAFE_SPEED, 0);
                }
                robot.move(0, 0, 0);
            }
        }

    }