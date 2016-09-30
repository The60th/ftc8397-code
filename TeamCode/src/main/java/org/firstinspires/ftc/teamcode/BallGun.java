package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by CanAdirondack on 9/22/2016.
 */
public class BallGun extends OpMode {
    DcMotor MotorOne;
    DcMotor MotorTwo;
    @Override
    public void init()
    {
        hardwareMap.logDevices();
        MotorOne = hardwareMap.dcMotor.get("M1");
        MotorTwo = hardwareMap.dcMotor.get("M2");
        MotorTwo.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop(){
        MotorOne.setPower(1);
        MotorTwo.setPower(1);
    }
}
