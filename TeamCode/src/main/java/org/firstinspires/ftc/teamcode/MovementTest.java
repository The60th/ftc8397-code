/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MovementTest: Teleop Tank", group="Test")
//@Disabled
public class MovementTest extends OpMode {
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor  = null;
    public DcMotor middleMotor  = null;
    double mainPower;
    double leftPower;
    double rightPower;



    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftMotor   = hardwareMap.dcMotor.get("leftMotor");
        rightMotor  = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor = hardwareMap.dcMotor.get("middleMotor");

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        middleMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.


    }


        @Override
        public void loop() {
        /*    //Forward power
        if (gamepad1.right_trigger > 0){
            mainPower = gamepad1.right_trigger;
        }
        //Reverse
        if(gamepad1.left_trigger > 0){
            mainPower = -1.0*(gamepad1.left_trigger);
        }
        //Turn Right
        if (gamepad1.left_stick_x > 0){
            rightPower = gamepad1.left_stick_x;
            rightMotor.setPower(mainPower - rightPower);
        //Turn Left
        } else if(gamepad1.left_stick_x < 0) {
            rightPower = Math.abs(gamepad1.left_stick_x);
            rightMotor.setPower(mainPower-leftPower);
        //Forward
        } else {
            leftMotor.setPower(mainPower);
            rightMotor.setPower(mainPower);
        }
        //Slide Right
        if(gamepad1.dpad_right == true){
            middleMotor.setPower(0.25);
        }
        //Slide Left
        if(gamepad1.dpad_left == true){
            middleMotor.setPower(-0.25);
        }
        if(gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }*/
        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);
        middleMotor.setPower(gamepad1.right_stick_x);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


}
