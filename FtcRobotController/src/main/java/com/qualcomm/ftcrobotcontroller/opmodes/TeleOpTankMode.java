package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class TeleOpTankMode  extends OpMode {

    final double Left_Spin = 1.0;
    final double Left_Spin_Stop = 0.5;

    final double Right_Spin = 0.0;
    final double Right_Spin_Stop = 0.5;

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor Arm1;
    DcMotor Arm2;
    Servo   Servo1;
    Servo   Servo2;
    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        Arm1 =hardwareMap.dcMotor.get("Arm1");
        Arm2 = hardwareMap.dcMotor.get("Arm2");
        Servo1 = hardwareMap.servo.get("Servo1");
        Servo2 = hardwareMap.servo.get("Servo2");
        //reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;
        float arm1 = gamepad2.left_stick_y;
        float arm2 = gamepad2.right_stick_y;

        //set the power of the motors with the gamepad values
        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);
        Arm2.setPower(arm2);
        Arm1.setPower(arm1);
        if(gamepad1.x) {
            Servo1.setPosition(Left_Spin);
            Servo2.setPosition(Right_Spin);
        }
        else {
            Servo1.setPosition(Left_Spin_Stop);
            Servo2.setPosition(Right_Spin_Stop);
        }




    }
}







