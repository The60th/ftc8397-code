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
    private Servo jewelArm;
    public Servo leftLowerClamp, leftUpperClamp, rightLowerClamp, rightUpperClamp, relicClamp;
    public DcMotor relicArm, liftArm;

    //May need different pos sets for servos depending on side.
    private final double CLAMP_CLOSE_POSITION = 1;
    private final double CLAMP_OPEN_POSTION = 0;
    private final double CLAMP_MID_POSTION = .5;

    private final double LINEAR_SLIDE_RAISE_POWER = 1;
    private final double LINEAR_SLIDE_LOWER_POWER = -1;

    private int nullPos = 1;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initHw();
    }
    public void init(HardwareMap ahwMap, float initGyroHeadingDegrees){
        super.init(ahwMap, initGyroHeadingDegrees);
        initHw();
    }
    private void initHw(){
        leftLinearSlide = hardwareMap.dcMotor.get("leftLinearSlide");
        rightLinearSlide = hardwareMap.dcMotor.get("rightLinearSlide");

        leftLowerClamp = hardwareMap.servo.get("leftLowerClamp");
        leftUpperClamp = hardwareMap.servo.get("leftUpperClamp");

        rightLowerClamp = hardwareMap.servo.get("rightLowerClamp");
        rightUpperClamp = hardwareMap.servo.get("rightUpperClamp");

        jewelArm = hardwareMap.servo.get("jewelArm");

        relicArm = hardwareMap.dcMotor.get("relicArm");
        liftArm = hardwareMap.dcMotor.get("liftArm");
        relicClamp = hardwareMap.servo.get("relicClamp");
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void closeLowerClamp(){
        leftLowerClamp.setPosition(1);
        rightLowerClamp.setPosition(0);
    }
    public void openLowerClamp(){
        leftLowerClamp.setPosition(.45);
        rightLowerClamp.setPosition(.75);
    }
    public void midPosLowerClamp(){
        leftLowerClamp.setPosition(0.55);
        rightLowerClamp.setPosition(0.45);
    }


    public void closeUpperClamp(){
        leftUpperClamp.setPosition(1);
        rightUpperClamp.setPosition(0);
    }
    public void openUpperClamp(){
        leftUpperClamp.setPosition(.2);
        rightUpperClamp.setPosition(.8);
    }
    public void midPosUpperClamp(){
        leftUpperClamp.setPosition(0.35);
        rightUpperClamp.setPosition(0.55);
    }




    public void liftArmDown(){
        leftLinearSlide.setPower(-1);
        rightLinearSlide.setPower(1);
    }
    public void liftArmUp (){
        leftLinearSlide.setPower(1);
        rightLinearSlide.setPower(-1);
    }
    public void liftArmStop (){
        leftLinearSlide.setPower(0);
        rightLinearSlide.setPower(0);
    }
    public void lowerJewelArm(){
        jewelArm.setPosition(.5);
    }
    public void raiseJewelArm(){
        jewelArm.setPosition(0);
    }

    public void relicArmOut(){
        relicArm.setPower(1);
    }
    public void relicArmIn(){
        relicArm.setPower(-1);
    }
    public void relicArmStop(){
        relicArm.setPower(0);
    }

    public void liftRelicArmUp(){
        liftArm.setPower(-.3);
    }
    public void liftRelicArmDown(){
        liftArm.setPower(.3);
    }
    public void liftRelicArmStop(){
        liftArm.setPower(0);
    }

    public void relicClampClose(){
        relicClamp.setPosition(.85);
    }
    public void relicClampOpen(){
        relicClamp.setPosition(0);
    }
    public void relicClampMid(){
        relicClamp.setPosition(.5);
    }
}
