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
    public DcMotor leftLinearSlide, rightLinearSlide;
    public Servo jewelArm, turnJewelArm;
    public Servo leftLowerClamp, leftUpperClamp, rightLowerClamp, rightUpperClamp, relicClamp;
    public DcMotor relicArm, liftArm;

    public final float BottomLeftClosePos = 1;
    public final float BottomLeftOpenPos = 0;

    public final float BottomRightClosePos = 0;
    public final float BottomRightOpenPos = 1;

    public final float TopLeftClosePos = 1;
    public final float TopLeftOpenPos = .40f;

    public final float TopRightClosePos = 0; //40
    public final float TopRightOpenPos =.60f;

    public float leftLowPos = 0;
    public float rightLowPos = 0;
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

        leftUpperClamp = hardwareMap.servo.get("leftLowerClamp");
        leftLowerClamp = hardwareMap.servo.get("leftUpperClamp");

        rightUpperClamp = hardwareMap.servo.get("rightLowerClamp");
        rightLowerClamp = hardwareMap.servo.get("rightUpperClamp");

        jewelArm = hardwareMap.servo.get("jewelArm");
        turnJewelArm = hardwareMap.servo.get("turnJewelArm");

        relicArm = hardwareMap.dcMotor.get("relicArm");
        liftArm = hardwareMap.dcMotor.get("liftArm");
        relicClamp = hardwareMap.servo.get("relicClamp");
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void closeUpperClamp(){
        leftUpperClamp.setPosition(TopLeftClosePos);
        rightUpperClamp.setPosition(TopRightClosePos);
    }
    public void openUpperClamp(){
        leftUpperClamp.setPosition(TopLeftOpenPos);
        rightUpperClamp.setPosition(TopRightOpenPos);
    }
    public void midPosUpperClamp(){
        leftUpperClamp.setPosition(0.55);
        rightUpperClamp.setPosition(0.55);
    }


    public void closeLowerClamp(){
        leftLowerClamp.setPosition(BottomLeftClosePos); //changed this number to fix the servo closing to far.
        rightLowerClamp.setPosition(BottomRightClosePos);
    }
    public void openLowerClamp(){
        leftLowerClamp.setPosition(BottomLeftOpenPos);
        rightLowerClamp.setPosition(BottomRightOpenPos);
    }
    public void midPosLowerClamp(){
        leftLowerClamp.setPosition(0.55);
        rightLowerClamp.setPosition(0.55);
    }




    public void liftArmDown(){
        leftLinearSlide.setPower(.1);
        rightLinearSlide.setPower(-.1);
    }
    public void liftArmUp (){
        leftLinearSlide.setPower(-.7);
        rightLinearSlide.setPower(.7);
    }
    public void liftArmStop (){
        leftLinearSlide.setPower(0);
        rightLinearSlide.setPower(0);
    }
    public void lowerJewelArm(){
        jewelArm.setPosition(1);
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
        relicClamp.setPosition(.95);
    }
    public void relicClampOpen(){
        relicClamp.setPosition(0);
    }
    public void relicClampMid(){
        relicClamp.setPosition(.5);
    }
}
