package org.firstinspires.ftc.teamcode.competition_in_work.teleop.old;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotNickBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;
import org.firstinspires.ftc.teamcode.mechbot.MechBotSensor;

/**
 * Created by FTC Team 8397 on 12/5/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Red hook Teleop", group="Comp")
public class TeleOpRedHook extends LoggingLinearOpMode {

    private MechBotRedHook bot = new MechBotRedHook();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    private float[] driveData = new float[6];

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

        while (opModeIsActive()){
            mechBotDriveControls.refreshGamepads(gamepad1,gamepad2);
            mechBotDriveControls.joyStickMecnumDriveComp(driveData); //Do an array fill by passing the array in, to prevent recreating the array.


            telemetry.addData("Joystick input: ","X: %.2f Y: %.2f A: %.2f", driveData[0],driveData[1],driveData[2]);
            telemetry.addData("Drive speeds input: ","X: %.2f Y: %.2f A: %.2f", driveData[3],driveData[4],driveData[5]);
            telemetry.update();

            while (opModeIsActive()){
                mechBotDriveControls.refreshGamepads(gamepad1,gamepad2);
                mechBotDriveControls.joyStickMecnumDriveComp(new float[6]);





                if(gamepad1.a){
                    bot.openLowerClamp();
                }
                else if(gamepad1.b){
                    bot.closeLowerClamp();
                }


                if(gamepad1.x){
                    bot.closeUpperClamp();
                }
                else if(gamepad1.y) {
                    bot.openUpperClamp();
                }else if(gamepad1.right_bumper || gamepad1.left_bumper){
                    bot.midPosUpperClamp();
                    bot.midPosLowerClamp();
                }


                if (gamepad2.dpad_up){
                    bot.liftArmUp();
                }
                else if (gamepad2.dpad_down){
                    bot.liftArmDown();
                }
                else{
                    bot.liftArmStop();
                }


                if(gamepad1.right_stick_y > .5){ //up
                    bot.lowerJewelArm();
                }else if(gamepad1.right_stick_y < -.5){ //down
                    bot.raiseJewelArm();
                }
            }
        }


        }


    }

