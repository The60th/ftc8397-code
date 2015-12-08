package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@SuppressWarnings("EmptyMethod")
public class ReUseAbleTest
{
    final double Left_Spin = 1.0;
    final double Left_Spin_Stop = 0.5;

    final double Right_Spin = 0.0;
    final double Right_Spin_Stop = 0.5;

    public final DcMotor rightRearMotor;
    public final DcMotor leftRearMotor;
    public final DcMotor leftMotor;
    public final DcMotor rightMotor;
    public final DcMotor midLeftMotor;
    public final DcMotor midRightMotor;
    Servo Servo1;
    Servo Servo2;




    public ReUseAbleTest(DcMotor left, DcMotor right, Servo Servo_left, Servo Servo_right)
    {
        leftMotor = left;
        midLeftMotor = left;
        leftRearMotor = left;
        rightMotor = right;
        midRightMotor = right;
        rightRearMotor = right;
        Servo1 = Servo_left;
        Servo2 = Servo_right;

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        midRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void MotorGo(double MOTOR_GO)
    {
        leftMotor.setPower(MOTOR_GO);
        midLeftMotor.setPower(MOTOR_GO);
        leftRearMotor.setPower(MOTOR_GO);

        rightMotor.setPower(MOTOR_GO);
        midRightMotor.setPower(MOTOR_GO);
        rightRearMotor.setPower(MOTOR_GO);
    }

    public void MotorHalt(double MOTOR_STOP)
    {
        leftMotor.setPower(MOTOR_STOP);
        midLeftMotor.setPower(MOTOR_STOP);
        leftRearMotor.setPower(MOTOR_STOP);

        rightMotor.setPower(MOTOR_STOP);
        midRightMotor.setPower(MOTOR_STOP);
        rightRearMotor.setPower(MOTOR_STOP);
    }

    public void ServoSpinLeft(double Left_Spin, double Left_Spin_Stop,
                              Gamepad gamepad1, Gamepad gamepad2)
    {
     if(gamepad1.x)
      {
       Servo1.setPosition(Left_Spin);
      }
      else
      {
       Servo1.setPosition(Left_Spin_Stop);
      }
    }

    public void ServoSpinRight(double Right_Spin, double Right_Spin_Stop,
                               Gamepad gamepad1, Gamepad gamepad2)
    {
     if(gamepad2.x)
      {
       Servo2.setPosition(Right_Spin);
      }
     else
      {
       Servo2.setPosition(Right_Spin_Stop);
      }
    }




}
