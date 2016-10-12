package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftccommon.DbgLog;
/**
 * Created by CanAdirondack on 10/6/2016.
 */

@Autonomous(name=" Autonomous : Test ", group="Test")

public class autonomous extends LinearOpMode {

    DcMotor one;
    DcMotor two;
    DcMotor three;
    DcMotor four;








    public void runOpMode() throws InterruptedException {
    one = hardwareMap.dcMotor.get("M1");
    two = hardwareMap.dcMotor.get("M2");
    three = hardwareMap.dcMotor.get("M3");
    four = hardwareMap.dcMotor.get("M4");


    two.setDirection(DcMotor.Direction.REVERSE);
    three.setDirection(DcMotor.Direction.REVERSE);


        one.setPower(1);
        three.setPower(1);
        sleep(2000);

        two.setPower(1);
        four.setPower(1);
        sleep(2000);
}
}



