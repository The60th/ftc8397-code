package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechbot.presupers_bot.MechBotRedHook;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 1/26/2018.
 */
@TeleOp(name = "Jewel arm Tester", group = "Tester")
//@Disabled
public class JewelArmTester extends LinearOpMode {
    private MechBotScranton bot = new MechBotScranton();
    double pos = .50;
    double UPpos = .50;
    UTILToggle bottomToggle = new UTILToggle();
    UTILToggle bottomToggleh = new UTILToggle();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.pivot.setPosition(pos);
        bot.arm.setPosition(UPpos);

        waitForStart();

        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive()){
            if (et.milliseconds() < 10) continue;
            et.reset();

            if(gamepad1.dpad_left){
                pos = pos-.002;
                if(pos <= 0) pos = 0;
                bot.pivot.setPosition(pos);
            }else if(gamepad1.dpad_right){
                pos = pos+.002;
                if(pos >= 1.0) pos = 1.0;
                bot.pivot.setPosition(pos);
            }

            if(gamepad1.dpad_down) {
                UPpos = UPpos - .002;
                if (UPpos <= 0) UPpos = 0;
                bot.arm.setPosition(UPpos);
            }
              else  if(gamepad1.dpad_up){
                UPpos = UPpos+.002;
                if(UPpos >= 1.0) UPpos = 1.0;
                bot.arm.setPosition(UPpos);
            }

            telemetry.addData("Pos: ", pos);
            telemetry.addData("UpPos: ", UPpos);
            telemetry.update();
        }
    }
}
