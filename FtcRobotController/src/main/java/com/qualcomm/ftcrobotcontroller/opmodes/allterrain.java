package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//Created by CanAdirondack on 11/25/2015.

public class allterrain extends OpMode{
    DcMotor rightRearMotor;
    DcMotor leftRearMotor;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor UpLeftMotor;
    DcMotor UpRightMotor;

    @Override

    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        UpLeftMotor =hardwareMap.dcMotor.get("midLeftMotor");
        UpRightMotor = hardwareMap.dcMotor.get("midRightMotor");
        leftRearMotor =hardwareMap.dcMotor.get("leftRearMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        UpRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override

    public void loop() {
        //set the power of the motors with the gamepad values
        double leftY = -gamepad1.left_stick_y * 100;
        double rightY = -gamepad1.right_stick_y * 100;
        //double UpPower = -.50;
        leftMotor.setPower(leftY/275);
        //midLeftMotor.setPower(leftY/275);
        leftRearMotor.setPower(leftY/275);

        rightMotor.setPower(rightY/275);
       // midRightMotor.setPower(rightY/275);
        rightRearMotor.setPower(rightY/275);

        if (gamepad1.left_bumper){
            UpLeftMotor.setPower(-1);
            UpRightMotor.setPower(-1);

        }
        else{
            UpLeftMotor.setPower(0);
            UpRightMotor.setPower(0);

        }


    }

}







