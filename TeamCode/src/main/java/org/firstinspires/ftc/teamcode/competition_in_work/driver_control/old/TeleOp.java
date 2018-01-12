package org.firstinspires.ftc.teamcode.competition_in_work.driver_control.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotNickBot;

/**
 * Created by FTC Team 8397 on 11/30/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Game TeleOP", group="Comp")
@Disabled
public class TeleOp extends LoggingLinearOpMode{

    private MechBotNickBot mechBot = new MechBotNickBot();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,mechBot);

    double kickerPos = .50;

    final double armModify = 0.0006;
    final double und_arm = 0.135;
    final double ovr_arm = 0.34;

    double armPos = .34; //.7 to high for all the way up. //.5 to far //3 lower then max. //4 still below max.

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        mechBot.init(hardwareMap);
        ElapsedTime et = new ElapsedTime();


        telemetry.addData("Ready to go: ","");
        telemetry.update();

        mechBot.updateOdometry();
        waitForStart();

        mechBot.raiseJewelArm();
        sleep(100);
        mechBot.breakJewelArm();
        telemetry.addData("Starting","");
        telemetry.update();

        while (opModeIsActive()) {
            mechBotDriveControls.refreshGamepads(gamepad1,gamepad2);
            mechBotDriveControls.joyStickMecnumDriveComp(new float[4]);

            //Extend block collector.
            if(gamepad1.dpad_up){
                mechBot.driveCollecter(MechBotNickBot.collecterStateValues.OUT);
            }else if(gamepad1.dpad_down){
                mechBot.driveCollecter(MechBotNickBot.collecterStateValues.IN);
            }else{
                mechBot.driveCollecter(MechBotNickBot.collecterStateValues.STOP);
            }
            if(gamepad1.right_stick_y > .5){
                mechBot.lowerJewelArm();
            }else if(gamepad1.right_stick_y < -.5){
                mechBot.raiseJewelArm();
            }else{
                mechBot.breakJewelArm();
            }

            //Drive block life. //Lifting the block up.
            if(gamepad2.dpad_up){
                mechBot.blockLiftMotor.setPower(1);
            }else if(gamepad2.dpad_down){
                mechBot.blockLiftMotor.setPower(-.5);
            }
            else{
                mechBot.blockLiftMotor.setPower(0);
            }


            //Swing block pusher. //Servo the pushes block down slide.
             if(gamepad2.right_trigger > .2){
               mechBot.kickerServo.setPosition(-.85);
           }
            else if(gamepad2.left_trigger > .2){
            mechBot.kickerServo.setPosition(1);
           }


           //Find the pos that is the wheel all the way drive so that the roller is up: -> max pos

            //Find the pos that is the wheel all the way drive so that the roller is down: -> min pos

           //Range within max-min pos:

            //Toggle up and down within range with modify function -> can't go over max pos or under min pos

            //Use max pos and base pos for init code.

            if(gamepad2.y  && armPos <= ovr_arm) {
                armPos += armModify;
            } else if (gamepad2.a  && armPos >= und_arm) {
                armPos -= armModify;
            }

           // mechBot.kickerServo.setPosition(kickerPos);

            mechBot.slideServo.setPosition(armPos);
           // if(et.milliseconds() > 100) {
             //   mechBot.raiseJewelArm();
            //}else{
             //   mechBot.breakJewelArm();
            //}

            telemetry.addData("Spinny servo pos: ", mechBot.slideServo.getPosition());

            //telemetry.addData("Gamepad 1: ","Left Stick Values X:"+gamepad1.left_stick_x+" Y: "+gamepad1.left_stick_y + " | Right Stick Values X:"+gamepad1.left_stick_x+" Y: "+gamepad1.left_stick_y);
           /// telemetry.addData("Gamepad 2: ","Left Stick Values X:"+gamepad2.right_stick_x+" Y: "+gamepad2.right_stick_y + " | Right Stick Values X:"+gamepad2.right_stick_x+" Y: "+gamepad2.right_stick_y);
            //telemetry.addData("Gamepad 1: " , gamepad1.toString());
            //telemetry.addData("Gamepad 2: " , gamepad2.toString());
            telemetry.update();
        }
    }
}
