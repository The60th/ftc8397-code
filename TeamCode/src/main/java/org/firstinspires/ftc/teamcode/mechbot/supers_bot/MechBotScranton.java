package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.openftc.hardware.rev.OpenRevDcMotorImplEx;

/**
 * Created by FTC Team 8397 on 3/6/2018.
 */

public class MechBotScranton extends MechBotSensorScranton {
    public OpenRevDcMotorImplEx leftIntake, rightIntake;
    public DcMotor relicArm;
    public DcMotor flipMotor;
    Servo   pincher, pincher2, pincher3, pincher4, kicker; //leftFlipper, rightFlipper,
    public Servo touchServo;
    public DigitalChannel touchSensor;
    public Servo pivot, arm, relicClaw,backStop;
    CRServo relicLift;
    public final double MAX_INTAKE_STALL_CURRENT_THRESHOLD = 6000;
    //public ServoImplEx leftFlipper, rightFlipper;
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
        flipMotor = hardwareMap.dcMotor.get("flipMotor");

        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        flipMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


//        leftFlipper = (ServoImplEx) hardwareMap.servo.get("leftFlipper");
//        rightFlipper = (ServoImplEx) hardwareMap.servo.get("rightFlipper");
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

        touchServo = hardwareMap.servo.get("touchServo");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");


    }

//    public void disableFlipPlate(){
//        leftFlipper.setPwmDisable();
//        rightFlipper.setPwmDisable();
//    }

    private float flipDownPos = .5f; // down
    private float flipUpPos = .93f; // up was .10
    private float pincherFullClosed = 0f;
    private float pincherIntakePos = .1f;
    private float pincherReleasePos = .1f;
    private float pincherFullFlat = .4f;
    private float kickerKickPos = .45f;
    private float kickerStoredPos = 0f;

    private final int FLIP_PLATE_UPTICKS_AUTO = 0;
    private final int FLIP_PLATE_DOWNTICKS_AUTO = 0;

    private boolean flipPlateActive = false;
    private float flipPlateTarget;

    private float touchServoOutPos = .44f;
    private float touchServoStorePos = .15f;

    protected void setTouchServoOut(){
        touchServo.setPosition(touchServoOutPos);
    }
    public void setTouchServoStore(){
        touchServo.setPosition(touchServoStorePos);
    }

  /*  public  void setFlipPlateDownwards(){
        ticks = flipMotor.getCurrentPosition();
        float  tickError = ticks - upticks;
        power = -tickError * COEFF;
        power = Math.min(power,1.0f);
        power = Math.max(power,-1.0f);
        flipMotor.setPower(power);


    }

    public  void setFlipPlateUpwards(){
        ticks = flipMotor.getCurrentPosition();
        float tickError = ticks - downticks;
        power = -tickError * COEFF;
        power = Math.min(power,1.0f);
        power = Math.max(power,-1.0f);
        flipMotor.setPower(power);

    }*/

  public void setFlipPosition (int flipPlatePos) {
      flipMotor.setTargetPosition(flipPlatePos);
      flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      flipMotor.setPower(.3);
  }
   /* public void setFlipPlateDownwards() {
        flipMotor.setTargetPosition(FLIP_PLATE_DOWNTICKS);
        flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipMotor.setPower(.3);
    }

    public void setFlipPlateUpwards() {
        flipMotor.setTargetPosition(FLIP_PLATE_UPTICKS);
        flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipMotor.setPower(.3);

    } */

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
        arm.setPosition(1.0);
    }

    public void setArmJewel() {
        arm.setPosition(.57);
    }

    public void setArmDrive(){
        arm.setPosition(1.0);
    }

    public void setPivotDrive(){
        pivot.setPosition(0.0);
    }
    public void setPivotStart() {
        pivot.setPosition(0.0);
    }

    public void setPivotEnd() {
        pivot.setPosition(.4);
    }

    public void knockPivotLeft() {
        pivot.setPosition(.49);
    }

    public void knockPivotRight() {
        pivot.setPosition(.27);
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
