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
@TeleOp(name=" Lift8020 : Test ", group="Test")
@Disabled

public class Lift8020 extends OpMode {

    CRServo Lift;

    @Override
    public void init() {
        hardwareMap.logDevices();
        Lift = hardwareMap.crservo.get("SL");



    }
    public void loop(){
        //Servo type now maters.
        if (gamepad1.left_trigger >= .5){

            Lift.setPower(1);

        }
        else if(gamepad1.right_trigger >= .5){

            Lift.setPower(-1);
        }
        else{
            Lift.setPower(0);

        }



    }
}

