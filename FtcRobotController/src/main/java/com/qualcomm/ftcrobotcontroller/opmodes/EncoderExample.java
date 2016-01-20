package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class EncoderExample extends LinearOpMode {
    DcMotor left;
    DcMotor right;

    @Override
    public void runOpMode() throws InterruptedException {
       left = hardwareMap.dcMotor.get("left");
       right = hardwareMap.dcMotor.get("right");

//Move 23inch*3.5 then turn to move 23*2.5

        left.setMode(DcMotorController.RunMode.RESET_ENCODERS); //Resets encoders
        right.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        while(left.getCurrentPosition() != 0 || right.getCurrentPosition() != 0) { //Ensures encoders are zero
            left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            waitOneFullHardwareCycle(); //Needed within all loops
        }
        left.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS); //Sets mode to use encoders
        right.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS); //setMode() is used instead of setChannelMode(), which is now deprecated
        waitForStart();
        left.setTargetPosition(1440); //Sets motor to move 1440 ticks (1440 is one rotation for Tetrix motors)
        //right.setTargetPosition(1440);
        left.setPower(.20);
        right.setPower(.20);
        while(Math.abs(left.getCurrentPosition()) < Math.abs(left.getTargetPosition()) ) { //While target has not been reached
            waitOneFullHardwareCycle(); //Needed within all loops
            telemetry.addData("Left Motor: ", left.getCurrentPosition());
            telemetry.addData("Right Motor: ", right.getCurrentPosition());
        }

        left.setPower(0);
        right.setPower(0);
        //To Do:

        //clear encoders
        //New drive commands foward for the drive
        //loop to check for drive amount
        //end loop stop motors save encoders values for later use?


        //clear encoders
        //new drive commands to turn robot using encoders
        //clear encoders
        //loop start and end ^^

        //clear encoders
        //new drive commands forward to color sensor
        //loop to drive before >=
        //end loop
        //save encoders valyes ro reuse.

        //check for lights using LED
        //if lights found "Do button press"(Would be a subclass that handles this like test for color drive ect.
        //run correct col.or
        //out print info.




    }
}
