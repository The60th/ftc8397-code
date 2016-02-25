package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Bradley on 2/25/16.
 */


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class test extends OpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    @Override
    public void init() {

        rightMotor = hardwareMap.dcMotor.get("RM1");
        leftMotor = hardwareMap.dcMotor.get("LM1");
        rightBackMotor = hardwareMap.dcMotor.get("RBM");
        leftBackMotor = hardwareMap.dcMotor.get("LBM");
    }
    @Override
    public void loop() {

    }


}
