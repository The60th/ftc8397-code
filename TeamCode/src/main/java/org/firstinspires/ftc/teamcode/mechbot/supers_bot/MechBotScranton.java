package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.openftc.hardware.rev.OpenRevDcMotorImplEx;

/**
 * Created by FTC Team 8397 on 3/6/2018.
 */

public class MechBotScranton extends MechBotSensorScranton {
    public OpenRevDcMotorImplEx leftIntake, rightIntake;
    public DcMotor relicArm;
    Servo leftFlipper, rightFlipper, backStop, pincher, pincher2, pincher3, pincher4, kicker;
    public Servo pivot, arm, relicClaw;
    CRServo relicLift;
    public final double MAX_INTAKE_STALL_CURRENT_THRESHOLD = 6000;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initHw();
    }

    public void init(HardwareMap ahwMap, float initGyroHeadingDegrees) {
        BetaLog.dd("MechBotIntake: ", "Init");
        super.init(ahwMap, initGyroHeadingDegrees);
        initHw();

    }

    public void initHw() {
        BetaLog.dd("MechBotIntake: ", "Init HW");

        leftIntake = new OpenRevDcMotorImplEx((DcMotorImplEx) hardwareMap.dcMotor.get("leftIntake"));
        rightIntake = new OpenRevDcMotorImplEx((DcMotorImplEx) hardwareMap.dcMotor.get("rightIntake"));

        relicArm = hardwareMap.dcMotor.get("relicArm");


        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");
        backStop = hardwareMap.servo.get("backStop");
        pincher = hardwareMap.servo.get("pincher"); // back right
        pincher2 = hardwareMap.servo.get("pincher2"); // back left
        pincher3 = hardwareMap.servo.get("pincher3"); // front left
        pincher4 = hardwareMap.servo.get("pincher4"); // front right
        kicker = hardwareMap.servo.get("kicker");

        pivot = hardwareMap.servo.get("pivot");
        arm = hardwareMap.servo.get("arm");

        relicClaw = hardwareMap.servo.get("relicClaw");
        relicLift = hardwareMap.crservo.get("relicLift");

    }

    float flipDownPos = .65f; // down
    float flipUpPos = .115f; // up was .10
    float pincherFullClosed = 0f;
    float pincherIntakePos = .1f;
    float pincherReleasePos = .1f;
    float pincherFullFlat = .4f;
    float kickerKickPos = .45f;
    float kickerStoredPos = 0f;

    public void setFlipPlateDownwards() {
        rightFlipper.setPosition(1 - flipDownPos);
        leftFlipper.setPosition(flipDownPos);
        backStop.setPosition(0);
        pincher.setPosition(pincherIntakePos); // .15 less is less
        pincher2.setPosition(1-pincherIntakePos); // .85 more is less
        pincher3.setPosition(pincherIntakePos); // .15 less is less
        pincher4.setPosition(1-pincherIntakePos); // .85 more is less
    }

    public void setFlipPlateUpwards() {
        rightFlipper.setPosition(1 - flipUpPos);
        leftFlipper.setPosition(flipUpPos);
        backStop.setPosition(.45);

    }

    public void setGlyphPincherClosed() {
        pincher.setPosition(pincherFullClosed); // .15 less is less
        pincher2.setPosition(1-pincherFullClosed); // .85 more is less
        pincher3.setPosition(pincherFullClosed); // .15 less is less
        pincher4.setPosition(1-pincherFullClosed); // .85 more is less
    } // in pos of each servo
    public void setGlyphPincherMidPos() {
        pincher.setPosition(pincherReleasePos); //more is more
        pincher2.setPosition(1-pincherReleasePos); // less is more
        pincher3.setPosition(pincherReleasePos); // more is more
        pincher4.setPosition(1-pincherReleasePos); // less is more
    }

    public void setGlyphPincherStartPos() { // this is  full flat
        pincher.setPosition(pincherFullFlat);
        pincher2.setPosition(1-pincherFullFlat);
        pincher3.setPosition(pincherFullFlat);
        pincher4.setPosition(1-pincherFullFlat);
    } // crypto position
   /* public void setTapGlyphPincher(){
        pincher.setPosition(pincherIntakePos); // .15 less is less
        pincher2.setPosition(1-pincherIntakePos); // .85 more is less
        pincher3.setPosition(pincherIntakePos); // .15 less is less
        pincher4.setPosition(1-pincherIntakePos); // .85 more is less
        pincher.setPosition(pincherFullClosed); // .15 less is less
        pincher2.setPosition(1-pincherFullClosed); // .85 more is less
        pincher3.setPosition(pincherFullClosed); // .15 less is less
        pincher4.setPosition(1-pincherFullClosed);
    }
    */
    public void setKickGlyph(){
        kicker.setPosition(kickerKickPos);


    }
     public void setRetractKicker(){
         kicker.setPosition(kickerStoredPos);

     }
    /* public void setGlyphPincher() { // this is half flat (no longer used because of always pinching when scoring)
        pincher.setPosition(.45);
        pincher2.setPosition(.85);
        pincher3.setPosition(.85);
        pincher4.setPosition(.45);
    } // crypto position*/


    public void setArmCube() {
        arm.setPosition(0.31);
    }

    public void setArmJewel() {
        arm.setPosition(.60);
    }

    public void setArmDrive(){
        arm.setPosition(.20);
    }

    public void setPivotDrive(){
        pivot.setPosition(.69);
    }
    public void setPivotStart() {
        pivot.setPosition(0.0);
    }

    public void setPivotEnd() {
        pivot.setPosition(0.61);
    }

    public void knockPivotLeft() {
        pivot.setPosition(0.50);
    }

    public void knockPivotRight() {
        pivot.setPosition(0.70);
    }

    public void setRelicClawClosed() {
        relicClaw.setPosition(.9);
    }

    public void setRelicClawMidPos() {
        relicClaw.setPosition(.3);
    }

    public void setRelicClawOpen() {
        relicClaw.setPosition(.3);
    }

    public void setRelicArmOut() {
        relicArm.setPower(1);
    }

    public void setRelicArmIn() {
        relicArm.setPower(-1);
    }

    public void setRelicArmStop() {
        relicArm.setPower(0);
    }

    public void setRelicLiftUp() {
        relicLift.setPower(-1);
    }

    public void setRelicLiftDown() {
        relicLift.setPower(1);
    }

    public void setRelicLiftStop() {
        relicLift.setPower(0);
    }

    public void setIntakeOn() {
        leftIntake.setPower(1);
        rightIntake.setPower(1);
    }

    public void setIntakeReverse() {
        leftIntake.setPower(-1);
        rightIntake.setPower(-1);
    }

    public void setIntakeOff() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public double getLeftIntakeCurrentDraw() { //6000 cut off.
        return leftIntake.getCurrentDraw();
    }

    public double getRightIntakeCurrentDraw() {
        return rightIntake.getCurrentDraw();
    }

    public boolean isIntakeStalled() {
        return (getLeftIntakeCurrentDraw() + getRightIntakeCurrentDraw() / 2.0) > MAX_INTAKE_STALL_CURRENT_THRESHOLD;
    }
}
