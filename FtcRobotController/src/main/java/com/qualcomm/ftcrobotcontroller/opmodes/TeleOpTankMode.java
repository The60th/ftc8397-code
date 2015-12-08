package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class TeleOpTankMode  extends OpMode {

    final double Spin = 1.0;
    final double Spin_Stop = 0.5;

    //final double Right_Spin = 0.0;
    //final double Right_Spin_Stop = 0.5;

    final double Up_Spin = 0.75;
    final double Down_Spin = 0;

    final double Up_Spin2 = 0.25;
    final double Down_Spin2 = 1;


    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor Arm1;
    DcMotor Arm2;
    Servo   Servo1;
    Servo   Servo2;
    Servo Servo3;
    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        Arm1 =hardwareMap.dcMotor.get("Arm1");
        Arm2 = hardwareMap.dcMotor.get("Arm2");
        Servo1 = hardwareMap.servo.get("Servo1");
        Servo2 = hardwareMap.servo.get("Servo2");
        Servo3 = hardwareMap.servo.get("Servo3");
        //reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
        float leftY = -gamepad1.left_stick_y * 100;
        float rightY = -gamepad1.right_stick_y * 100;
        float arm1 = -gamepad2.left_stick_y * 100;
        float arm2 = -gamepad2.right_stick_y * 100;

        //set the power of the motors with the gamepad values
        leftMotor.setPower(leftY/200);
        rightMotor.setPower(rightY/200);
        Arm2.setPower(arm2/350);
        Arm1.setPower(arm1/350);
        if(gamepad1.x)
        {
            Servo1.setPosition(Spin);
            //Servo2.setPosition(Right_Spin);
        }
        else
        {
            Servo1.setPosition(Spin_Stop);
           // Servo2.setPosition(Right_Spin_Stop);
        }

        if(gamepad1.y)
        {
            Servo2.setPosition(Up_Spin);
            Servo3.setPosition(Up_Spin2);

        }
        else
        {
            Servo2.setPosition(Down_Spin);
            Servo3.setPosition(Down_Spin2);

        }





    }
}







