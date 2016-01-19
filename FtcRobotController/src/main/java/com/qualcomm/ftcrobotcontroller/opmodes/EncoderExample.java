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
        right.setTargetPosition(1440);
        left.setPower(.5);
        right.setPower(.5);
        while(left.getCurrentPosition() < left.getTargetPosition() || right.getCurrentPosition() < right.getTargetPosition()) { //While target has not been reached
            waitOneFullHardwareCycle(); //Needed within all loops
        }
        left.setPower(0);
        right.setPower(0);
    }
}
