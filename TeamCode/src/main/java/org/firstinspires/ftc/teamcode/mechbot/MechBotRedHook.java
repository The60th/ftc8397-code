package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 12/5/2017.
 */

public class MechBotRedHook extends MechBotSensor{
    private DcMotor leftLinearSlide, rightLinearSlide;
    private CRServo jewelArm;
    private Servo leftLowerClamp, leftUpperClamp, rightLowerClamp, rightUpperClamp;

    //May need different pos sets for servos depending on side.
    private final double CLAMP_CLOSE_POSITION = 1;
    private final double CLAMP_OPEN_POSTION = 0;
    private final double CLAMP_MID_POSTION = .5;

    private final double LINEAR_SLIDE_RAISE_POWER = 1;
    private final double LINEAR_SLIDE_LOWER_POWER = -1;

    private int nullPos = 1;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        leftLinearSlide = hardwareMap.dcMotor.get("leftLinearSlide");
        rightLinearSlide = hardwareMap.dcMotor.get("rightLinearSlide");

        leftLowerClamp = hardwareMap.servo.get("leftLowerClamp");
        leftUpperClamp = hardwareMap.servo.get("leftUpperClamp");

        rightLowerClamp = hardwareMap.servo.get("rightLowerClamp");
        rightUpperClamp = hardwareMap.servo.get("rightUpperClamp");

        jewelArm = hardwareMap.crservo.get("jewelArm");

    }

    public void raiseLinearSlideLift(){
        leftLinearSlide.setPower(1);
        rightLinearSlide.setPower(1);
    }
    public void lowerLinearSlideLift(){
        leftLinearSlide.setPower(-1);
        rightLinearSlide.setPower(-1);
    }

    public void closeLowerClamp(){
        leftLowerClamp.setPosition(nullPos);
        rightLowerClamp.setPosition(nullPos);
    }

    public void openLowerClamp(){
        leftLowerClamp.setPosition(nullPos);
        rightLowerClamp.setPosition(nullPos);
    }

    public void closeUpperClamp(){
        leftUpperClamp.setPosition(nullPos);
        rightUpperClamp.setPosition(nullPos);
    }

    public void openUpperClamp(){
        leftUpperClamp.setPosition(nullPos);
        rightUpperClamp.setPosition(nullPos);
    }


    public void lowerJewelArm(){
        jewelArm.setPower(.5);
    }
    public void raiseJewelArm(){
        jewelArm.setPower(-.5);
    }
    public void breakJewelArm(){
        jewelArm.setPower(0);
    }
}
