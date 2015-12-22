package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by CanAdirondack on 12/22/2015.
 */

public class autobluep1ramp1noD extends LinearOpMode {

    DcMotor rightRearMotor ;
    DcMotor leftRearMotor ;
    DcMotor leftMotor ;
    DcMotor rightMotor ;
    DcMotor UpLeftMotor ;
    DcMotor UpRightMotor ;

    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();
        while(opModeIsActive()) {
            rightRearMotor.setPower(.5);
            leftRearMotor.setPower(.5);
            rightMotor.setPower(.5);
            leftMotor.setPower(.5);
            Thread.sleep(1000);

            rightRearMotor.setPower(-.5);
            leftRearMotor.setPower(.5);
            rightMotor.setPower(-.5);
            leftMotor.setPower(.5);
            Thread.sleep(200);

            rightRearMotor.setPower(.5);
            leftRearMotor.setPower(.5);
            rightMotor.setPower(.5);
            leftMotor.setPower(.5);
            Thread.sleep(4000);
        }

    }




}
