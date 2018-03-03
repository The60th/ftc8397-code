package org.firstinspires.ftc.teamcode.mechbot.presupers_bot;

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


        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void closeUpperClamp(){
        leftUpperClamp.setPosition(.37); //Left upper all the way in. 0 was 39
        rightUpperClamp.setPosition(.53);  //Right upper all the way in. 1 was 51
    }
    public void openUpperClamp(){
        leftUpperClamp.setPosition(.50); //Left upper all the way out. 1 //70
        rightUpperClamp.setPosition(.40); //Right upper all the way out. 0 //20
    }
    public void midPosUpperClamp(){
        leftUpperClamp.setPosition(0.45); // This works as of 1/26/18.
        rightUpperClamp.setPosition(0.45); // This works as of 1/26/18.
    }
    public void fullOpenUpperClamp(){
        leftUpperClamp.setPosition(.70); //Left upper all the way out. 1 //70
        rightUpperClamp.setPosition(.20); //Right upper all the way out. 0 //20
    }

    public void closeLowerClamp(){
        leftLowerClamp.setPosition(.90);  //Left lower all the way in. 1
        rightLowerClamp.setPosition(.10); //Right lower all the way in. 0
    }
    public void openLowerClamp(){
        leftLowerClamp.setPosition(0); //Left lower all the way out. 0
        rightLowerClamp.setPosition(1); //Right lower all the way out. 1
    }
    public void midPosLowerClamp(){
        leftLowerClamp.setPosition(.55); // This Works.
        rightLowerClamp.setPosition(.40); // This Works.
    }




    public void liftArmDown(){
        leftLinearSlide.setPower(-.25);
        rightLinearSlide.setPower(-.25);
    }
    public void liftArmUp (){
        leftLinearSlide.setPower(1.0);
        rightLinearSlide.setPower(1.0);
    }
    public void liftArmStop (){
        leftLinearSlide.setPower(0);
        rightLinearSlide.setPower(0); //
    }



    public void lowerJewelArm(){
        jewelArm.setPosition(1);
    }
    public void raiseJewelArm(){
        jewelArm.setPosition(0);
    }

    public void relicArmOut(){
        relicArm.setPower(-1);
    }
    public void relicArmIn(){
        relicArm.setPower(1);
    }
    public void relicArmStop(){
        relicArm.setPower(0);
    }

    public void liftRelicArmUp(){
        liftArm.setPower(-.6/relicArmModify);
    }
    public void liftRelicArmDown(){
        liftArm.setPower(.6/relicArmModify);
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
        relicClamp.setPosition(.60);
    }

    public float relicArmModify = 1.0f;
}
