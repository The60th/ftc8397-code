package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Team 8397 on 2/22/2018.
 */

public class MechBotIntake extends MechBot{

    public DcMotor rightIntake;
    public DcMotor  leftIntake;
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        rightIntake = hardwareMap.dcMotor.get("RI");
        leftIntake = hardwareMap.dcMotor.get("LI");

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
