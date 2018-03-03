package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

 /**
 * Created by FTC Team 8397 on 2/23/2018.
 */

public class MechBotFlip extends MechBotIntake {
    Servo leftFlipper, rightFlipper, backStop, pincher;
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");
        backStop = hardwareMap.servo.get("backStop");
        pincher = hardwareMap.servo.get("pincher");

        //leftFlipper.setDirection();
    }
     float flipUpPos = .55f;
     float flipDownPos = .05f;
    public void flipPlateDownwards(){
        leftFlipper.setPosition(flipDownPos);
        rightFlipper.setPosition(1-flipDownPos);
        backStop.setPosition(0);
    }

    public void flipPlateUpwards(){
        leftFlipper.setPosition(flipUpPos);
        rightFlipper.setPosition(1-flipUpPos);
        backStop.setPosition(.45);

    }
    public void pinchGlyph(){
        pincher.setPosition(0);
    }
    public void setGlyph(){
        pincher.setPosition(.15);
    }
    public void startPos(){
        pincher.setPosition(.45);
    }
}
