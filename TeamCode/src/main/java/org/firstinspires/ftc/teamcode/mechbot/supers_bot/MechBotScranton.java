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
    Servo leftFlipper, rightFlipper, backStop, pincher, pincher2;
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
        pincher = hardwareMap.servo.get("pincher");
        pincher2 = hardwareMap.servo.get("pincher2");

        pivot = hardwareMap.servo.get("pivot");
        arm = hardwareMap.servo.get("arm");

        relicClaw = hardwareMap.servo.get("relicClaw");
        relicLift = hardwareMap.crservo.get("relicLift");

    }

    float flipUpPos = .55f;
    float flipDownPos = .05f;

    public void setFlipPlateDownwards() {
        leftFlipper.setPosition(flipDownPos);
        rightFlipper.setPosition(1 - flipDownPos);
        backStop.setPosition(0);
        pincher.setPosition(.15);
        pincher2.setPosition(.85);
    }

    public void setFlipPlateUpwards() {
        leftFlipper.setPosition(flipUpPos);
        rightFlipper.setPosition(1 - flipUpPos);
        backStop.setPosition(.45);

    }

    public void setGlyphPincherClosed() {
        pincher.setPosition(.01);
        pincher2.setPosition(.99);
    } // in pos of each servo
    public void setGlyphPincherMidPos() {
        pincher.setPosition(.15);
        pincher2.setPosition(.85);
    } // out pos of each servo

    public void setGlyphPincherStartPos() { // this is full flat
        pincher.setPosition(.45);
        pincher2.setPosition(.60);
    } // crypto position
    public void setGlyphPincher() { // this is full flat
        pincher.setPosition(.45);
        pincher2.setPosition(.85);
    } // crypto position

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
