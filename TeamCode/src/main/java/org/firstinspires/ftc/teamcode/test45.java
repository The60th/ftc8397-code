package org.firstinspires.ftc.teamcode;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftccommon.DbgLog;


/**
 * Created by CanAdirondack on 9/30/2016.
 */
@TeleOp(name=" Test45 : Test ", group="Test")

public class test45  extends OpMode {

    DcMotor one;
    DcMotor two;
    DcMotor three;
    DcMotor four;

    @Override
    public void init() {

        one = hardwareMap.dcMotor.get("M1");
        two = hardwareMap.dcMotor.get("M2");
        three = hardwareMap.dcMotor.get("M3");
        four = hardwareMap.dcMotor.get("M4");


        two.setDirection(DcMotor.Direction.REVERSE);
        three.setDirection(DcMotor.Direction.REVERSE);

    }
    public void loop(){

        double F_B_Drive = gamepad1.right_stick_y;
        double L_R_Drive = gamepad1.right_stick_x;
        double TurnDrive = gamepad1.left_stick_x;

        if ( F_B_Drive <= -.05) {
        //Forward drive to the north east.
            one.setPower(.5);
            three.setPower(.5);
            two.setPower(.5);
            four.setPower(.5);
        }

        else if ( F_B_Drive >= .05) {
        //Backwards drive to the south west.
            one.setPower(-.5);
            three.setPower(-.5);
            two.setPower(-.5);
            four.setPower(-.5);
            }
        else if ( L_R_Drive <= -.05) {
                two.setPower(.5);
                four.setPower(.5);
                one.setPower(-.5);
                three.setPower(-.5);
            }
            else if ( L_R_Drive >= .05) {

            two.setPower(-.5);
            four.setPower(-.5);
            one.setPower(.5);
            three.setPower(.5);
        }
            else if (TurnDrive >= .05){

            one.setPower(-.5);
            two.setPower(.5);
            three.setPower(.5);
            four.setPower(-.5);
        }
        else if (TurnDrive <= -.05){

            one.setPower(.5);
            two.setPower(-.5);
            three.setPower(-.5);
            four.setPower(.5);
        }
        else {

            one.setPower(0);
            two.setPower(0);
            three.setPower(0);
            four.setPower(0);
        }


    }



    }


