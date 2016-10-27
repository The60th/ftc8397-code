package org.firstinspires.ftc.teamcode;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftccommon.DbgLog;


/**
 * Created by CanAdirondack on 9/30/2016.
 */
@TeleOp(name=" BigBallLift : Test ", group="Test")

public class BigBallLift  extends OpMode {

    Servo Lift;
    Servo Grabber;

    @Override
    public void init() {
        hardwareMap.logDevices();
        Lift = hardwareMap.servo.get("SL");
        Grabber = hardwareMap.servo.get("SG");


    }
    public void loop(){
        //Servo type now maters.
        if (gamepad1.left_trigger >= .5){

            Lift.setPosition(0);

        }
        else if(gamepad1.right_trigger >= .5){

            Grabber.setPosition(1);
        }
        else if (gamepad1.left_bumper){

            Lift.setPosition(1);
        }
        else if (gamepad1.right_bumper){

            Grabber.setPosition(0);
        }
        else{

            Lift.setPosition(1);
            Grabber.setPosition(0);
        }



    }
}


