package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

/**
 * Created by CanAdirondack on 9/29/2016.
 */

@TeleOp(name=" BallGun2 : Test ", group="Test")

public class BallGun2  extends OpMode {

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    @Override
    public void init() {

        leftMotor   = hardwareMap.dcMotor.get("leftMotor");
        rightMotor  = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);




    }

    @Override
    public void loop() {




        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }

    }