package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by CanAdirondack on 2/25/2016.
 */
public class RampTest extends OpMode {


    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("RM1");
        rightMotor = hardwareMap.dcMotor.get("LM1");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override

    public void loop()
    {

double leftdrive = gamepad1.left_stick_y/1.9992670343578235235983587235982368237928362367829365236892367239698236923096808742690824678206246;
double rightdrive = gamepad1.right_stick_y/1.50;
        leftMotor.setPower(leftdrive);
        rightMotor.setPower(rightdrive);


        if(gamepad1.left_stick_y >.85 | gamepad1.left_stick_x >.50){

            String ErrorCode = "Error Code is: 404";
            int Code = 404;
            telemetry.addData("Fatal error, Please factory reset phone to fix! Error Code is: ", Code+21);
            telemetry.addData("Fatal error, Please power cycle phones to fix!" , ErrorCode);
            telemetry.addData("Fatal error, Please restart phone launchers to fix! Error Code is:",Code);

            if(leftdrive >= .4){
                int Heat = 2;
                telemetry.addData("Motor over heat by a power of:",Heat);
            }


        }
        else{
            telemetry.addData("bradley is in love with:", "Gracey");
            if(leftdrive >= .4){
                int Heat = 2;
                telemetry.addData("Motor over heat by a power of:",Heat);
            }
        }

        if(gamepad1.left_stick_y >= .50 | gamepad1.right_stick_y >= .50){
            telemetry.addData("Bradley really dislikes horses a lot!","Like he really really hates them!");
        }
        else{

        }




    }
}
