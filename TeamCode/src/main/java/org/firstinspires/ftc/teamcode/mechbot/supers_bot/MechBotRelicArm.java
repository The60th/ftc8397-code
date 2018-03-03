package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 3/1/2018.
 */

public class MechBotRelicArm extends MechBotJewelArm {
    public Servo claw, clawPivot;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        claw = hardwareMap.servo.get("claw");
        //Relic grabbing claw.
        //right = closed 1.0 closed
        //left = way to open.

        clawPivot = hardwareMap.servo.get("clawPivot");
        //right = up 1.0
        //left = down 0.0
    }

}
