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
    DcMotor Lift;
    DcMotor Grabber;

    @Override
    public void init() {

        one = hardwareMap.dcMotor.get("M1");
        two = hardwareMap.dcMotor.get("M2");
        three = hardwareMap.dcMotor.get("M3");
        four = hardwareMap.dcMotor.get("M4");
        Lift = hardwareMap.dcMotor.get("SL");
        Grabber = hardwareMap.dcMotor.get("SG");


        two.setDirection(DcMotor.Direction.REVERSE);
        three.setDirection(DcMotor.Direction.REVERSE);

    }
    public void loop(){

        double F_B_Drive = gamepad1.right_stick_y;
        double L_R_Drive = gamepad1.right_stick_x;
        double TurnDrive = gamepad1.left_stick_x;

        if ( F_B_Drive <= -.05) {
        //Forward drive.
            two.setPower(-.5);
            four.setPower(-.5);
            one.setPower(.5);
            three.setPower(.5);
        }

        else if ( F_B_Drive >= .05) {
        //Backwards drive.
            two.setPower(.5);
            four.setPower(.5);
            one.setPower(-.5);
            three.setPower(-.5);
            }
        else if ( L_R_Drive <= -.05) {
            //Left drive. or back

            one.setPower(.5);
            three.setPower(.5);
            two.setPower(.5);
            four.setPower(.5);
            }
            else if ( L_R_Drive >= .05) {
            //Right drive. or forward
            one.setPower(-.5);
            three.setPower(-.5);
            two.setPower(-.5);
            four.setPower(-.5);

        }
            else if (TurnDrive >= .05){

            one.setPower(.5);
            two.setPower(-.5);
            three.setPower(-.5);
            four.setPower(.5);
        }
        else if (TurnDrive <= -.05){

            one.setPower(-.5);
            two.setPower(.5);
            three.setPower(.5);
            four.setPower(-.5);
        }
        else if (gamepad1.left_trigger >= .5){

            Lift.setPower(1);

        }
        else if(gamepad1.right_trigger >= .5){

            Grabber.setPower(-1);
        }
        else {

            one.setPower(0);
            two.setPower(0);
            three.setPower(0);
            four.setPower(0);
            Lift.setPower(0);
            Grabber.setPower(0);
        }


    }



}


