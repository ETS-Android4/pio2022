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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


/**
 * This file tests the proto competition robot's movement
 *
 * This particular OpMode just executes movement for a four wheeled robot.
 */

@TeleOp(name="Lifter Movement", group="Proto Lifter")

public class ProtoLifterMove extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lifter;
    private CRServo bucket;

    //For performance measuring
    private double prevElapsedTime = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //lifter = hardwareMap.get(DcMotor.class, "lifter_motor");
        bucket = hardwareMap.get(CRServo.class, "bucket_servo");

        //Sensors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        prevElapsedTime = 0;

        // run until the end of the match (d river presses STOP)
        while (opModeIsActive()) {
            double servoPow;
            //Bucket Control
            if(gamepad1.dpad_right)servoPow = -0.3;
            else if(gamepad1.dpad_left)servoPow = -0.3;
            else servoPow = 0;

            // Lifter control
            //if(gamepad1.dpad_up)lifter.setPower(-1);
            //else if(gamepad1.dpad_down)lifter.setPower(1);
            //else lifter.setPower(0);

            bucket.setPower(servoPow);
            
            // Show the elapsed game time, performance, and wheel power.
            telemetry.addData("Status", "\n\tRun Time: " + runtime.toString() + "\n\tTPS: %.2f", 1/(getRuntime()-prevElapsedTime));
            telemetry.addData("Motors", "\n\tLM:(%.2f)\tBS:(%.2f)",0.0, servoPow);
            telemetry.update();
            prevElapsedTime = getRuntime();
        }
    }
}
