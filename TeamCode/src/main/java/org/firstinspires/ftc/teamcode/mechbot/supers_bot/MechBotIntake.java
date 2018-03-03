package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechbot.MechBot;

/**
 * Created by FTC Team 8397 on 2/22/2018.
 */

public class MechBotIntake extends MechBot {

    public DcMotor leftIntake, rightIntake;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
