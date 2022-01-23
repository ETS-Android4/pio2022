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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name="Autonomous with Camera", group="Comp Robot")

public class AutonomousA extends LinearOpMode{

    /* Declare OpMode members. */
    CompRobot robot   = new CompRobot();   // Use the common comp robot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    double[][] duckPositions = {{0.0,0.0},{0.0,0.0},{0.0,0.0}};
    int level = 0, consDetections = 0;

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     STRAFE_SPEED  = 0.5;
    private DistanceSensor frontRange, rightRange;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initVuforia(hardwareMap);

        //you can use this as a regular distance sensor
        frontRange = hardwareMap.get(DistanceSensor.class, "front_distance");
        rightRange = hardwareMap.get(DistanceSensor.class, "right_distance");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");//
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Start near warehouse
        while(opModeIsActive())
        {
            sleep(500);
            //Duck Detection
            while(consDetections < 5){
                boolean success = false;
                List<Recognition> objects = robot.runTFod();
                sleep(100);
                if(objects != null){
                    for(Recognition recognition : objects){
                        if(recognition.getLabel().equals("Duck")){
                            success = true;
                            if(recognition.getLeft() > 250) {
                                if(level != 3) consDetections = 0;
                                else consDetections++;
                                level = 3;
                            }
                            else {
                                if(level != 2) consDetections = 0;
                                else consDetections++;
                                level = 2;
                            }
                        }
                    }
                }
                if(!success){
                    if(level != 1) consDetections = 0;
                    else consDetections++;
                    level = 1;
                }
            }
            telemetry.addData("Level", level);
            telemetry.update();

            //Move until 30cm away from wall
            while(frontRange.getDistance(DistanceUnit.CM)<30)
            {
                telemetry.addData("Distance", frontRange.getDistance(DistanceUnit.CM));
                telemetry.update();
                robot.move(-1,0,0);
            }
            //Move left for 2 seconds
            runtime.reset();
            while(runtime.seconds()<2)
            {
                robot.move(0,-1,0);
            }
            //Move away from wall until 42cm away
            runtime.reset();
            while(frontRange.getDistance(DistanceUnit.CM)<42 )
            {
                telemetry.addData("Distance", frontRange.getDistance(DistanceUnit.CM));
                telemetry.update();
                robot.move(-1,0,0);
            }
            robot.move(0,0,0);

            robot.lifterPID.minError = 30;
            if(level == 1){
                //Move lifter up
                robot.lifter(true,false,false,false,false);
                while(Math.abs(robot.lifterPID.error(robot.lifterMotor.getCurrentPosition())) > robot.lifterPID.minError){
                    robot.lifter(true,false,false,false,false);
                }
                runtime.reset();
                //move bucket forward
                while(runtime.seconds() < 1) {
                    robot.lifter(false, false, false, true, false);
                }
                runtime.reset();
                //move bucket back
                while(runtime.seconds() < 1) {
                    robot.lifter(false, false, true, false, false);
                }
            }
            else if(level == 2){
                robot.lifter(true,false,false,false,true);
                while(Math.abs(robot.lifterPID.error(robot.lifterMotor.getCurrentPosition())) > robot.lifterPID.minError){
                    robot.lifter(true,false,false,false,false);
                }
                runtime.reset();
                while(runtime.seconds() < 1) {
                    robot.lifter(false, false, true, false, false);
                }
                runtime.reset();
                while(runtime.seconds() < 1) {
                    robot.lifter(false, false, false, true, false);
                }
            }
            else
            {
                robot.lifter(true,false,false,false,false);
                while(Math.abs(robot.lifterPID.error(robot.lifterMotor.getCurrentPosition())) > robot.lifterPID.minError){
                    robot.lifter(true,false,false,false,false);
                }
                runtime.reset();
                while(runtime.seconds() < 1) {
                    robot.lifter(false, false, true, false, false);
                }
                runtime.reset();
                while(runtime.seconds() < 1) {
                    robot.lifter(false, false, false, true, false);
                }
            }
            robot.lifter(false,true,false,false,false);
            //move lift down
            while(Math.abs(robot.lifterPID.error(robot.lifterMotor.getCurrentPosition())) > robot.lifterPID.minError){
                robot.lifter(false,false,false,false,false);
            }

            //Move back to clear goal
            runtime.reset();
            while(runtime.seconds() < 0.3)
            {
                robot.move(1,0,0);
            }
            //Move towards right wall until 10cm close
            while(rightRange.getDistance(DistanceUnit.CM) > 10){
                telemetry.addData("Distance", rightRange.getDistance(DistanceUnit.CM));
                telemetry.update();
                robot.move(0, 1, 0);
            }
            //Adjust robot to be in square
            while(frontRange.getDistance(DistanceUnit.CM)<50)
            {
                telemetry.addData("Distance", frontRange.getDistance(DistanceUnit.CM));
                telemetry.update();
                robot.move(-1,0,0);
            }
            robot.move(0,0,0);
            stop();
        }


    }
}

