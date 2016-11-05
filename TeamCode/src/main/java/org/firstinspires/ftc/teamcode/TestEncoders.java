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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.OmniBot;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TestEncoders", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class TestEncoders extends LinearOpMode {

  OmniBot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new OmniBot();
        robot.init(hardwareMap);
        int [] maxSpeeds = robot.getMaxDriveTicksPerSec();
        telemetry.addData("max1 ",maxSpeeds[0]);
        telemetry.addData("max2 ",maxSpeeds[1]);
        telemetry.addData("max3 ",maxSpeeds[2]);
        telemetry.addData("max4 ",maxSpeeds[3]);
        telemetry.update();
        waitForStart();
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //telemetry.addData("one: ", robot.one.getCurrentPosition());
        //telemetry.update();

        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime et = new ElapsedTime();
        et.reset();
        double scale = robot.setStraightDriveSpeed(0,20);
        telemetry.addData("scale = ", scale);
        telemetry.update();
        while(opModeIsActive()&& et.seconds() <= 10.0) idle ();
        robot.setDrivePower(0,0,0);
        telemetry.addData("Ticks one ", robot.one.getCurrentPosition());
        telemetry.addData("Ticks two ", robot.two.getCurrentPosition());
        telemetry.addData("Ticks three ", robot.three.getCurrentPosition());
        telemetry.addData("Ticks four ", robot.four.getCurrentPosition());
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        /*while (opModeIsActive()) {

            if (gamepad1.a)
                break;
            telemetry.addData("one: ", robot.one.getCurrentPosition());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }*/
        while (opModeIsActive()) {
            idle();
        }
    }
}
