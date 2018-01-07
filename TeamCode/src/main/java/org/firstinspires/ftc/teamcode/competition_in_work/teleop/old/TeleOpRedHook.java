package org.firstinspires.ftc.teamcode.competition_in_work.teleop.old;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotNickBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;
import org.firstinspires.ftc.teamcode.mechbot.MechBotSensor;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 12/5/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Red hook Teleop", group="Comp")
public class TeleOpRedHook extends LoggingLinearOpMode {

    private MechBotRedHook bot = new MechBotRedHook();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    private float[] driveData = new float[6];
    UTILToggle topToggle = new UTILToggle();    // Slows down drivetrain when on
    boolean topStatus = false;
    GrabberState topState = GrabberState.OPEN;
    UTILToggle bottomToggle = new UTILToggle();    // Slows down drivetrain when on
    boolean bottomStatus = false;
    GrabberState bottomState = GrabberState.OPEN;

    enum GrabberState{CLOSED,OPEN}
    //Button logic.
    //Bottom can't close till top opens.
    //Top can't close till bottom closes.

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);


        ElapsedTime et = new ElapsedTime();

        telemetry.addData("Ready to start TeleOp, waiting for starting button.","");
        telemetry.update();
        bot.updateOdometry();

        waitForStart();
        telemetry.addData("Started TeleOp","");
        telemetry.update();
        bot.raiseJewelArm();
        bot.openLowerClamp();
        bot.openUpperClamp();
        while (opModeIsActive()) {
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompQuad(driveData); //Do an array fill by passing the array in, to prevent recreating the array.

            telemetry.addData("Joystick input: ", "X: %.2f Y: %.2f A: %.2f", driveData[0], driveData[1], driveData[2]);
            telemetry.addData("Drive speeds input: ", "X: %.2f Y: %.2f A: %.2f", driveData[3], driveData[4], driveData[5]);
            telemetry.update();


            //TODO
            //REMAP SERVO CONTROLS
            //Servo controls and servo functions are not reflective of what they really do.
            //Please test them to fix it.

            //Toggle should be: mid-> close
            //And secondary button for open

            if (bottomToggle.status(gamepad1.y) == UTILToggle.Status.COMPLETE) {
                if(!topStatus) {
                    bot.closeUpperClamp();
                    topState = GrabberState.CLOSED;
                }
                else if(topStatus) {
                    bot.openUpperClamp();
                    topState = GrabberState.OPEN;
                }
                topStatus = !topStatus;
            }

            if (topToggle.status(gamepad1.a) == UTILToggle.Status.COMPLETE) {
                if(!bottomStatus) {
                    bot.closeLowerClamp();
                    bottomState = GrabberState.CLOSED;
                }
                else if(bottomStatus) {
                    bot.openLowerClamp();
                    bottomState = GrabberState.OPEN;
                }
                bottomStatus =!bottomStatus;
            }

            if (gamepad1.right_bumper){
                bot.midPosLowerClamp();
                bot.midPosUpperClamp();
            }


            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                bot.liftArmUp();
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                bot.liftArmDown();
            } else {
                bot.liftArmStop();
            }


            if (gamepad1.right_stick_y > .5) { //up
                bot.lowerJewelArm();
            } else if (gamepad1.right_stick_y < -.5) { //down
                bot.raiseJewelArm();
            }

            if (gamepad2.y) {
                bot.relicArmOut();
            } else if (gamepad2.a) {
                bot.relicArmIn();
            } else {
                bot.relicArmStop();
            }

            if (gamepad2.right_bumper) {
                bot.liftRelicArmUp();
            } else if (gamepad2.right_trigger > .05) {
                bot.liftRelicArmDown();
            } else {
                bot.liftRelicArmStop();
            }
            if (gamepad2.left_bumper) {
                bot.relicClampOpen();
            }
            if (gamepad2.left_trigger > .05) {
                bot.relicClampClose();
            }
            if(gamepad2.a) {
                bot.relicClampMid();
            }
        }
        }


        }




