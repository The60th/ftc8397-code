package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotNickBot;

/**
 * Created by FTC Team 8397 on 9/29/2017.
 */
@TeleOp(name="Debug teleOP", group="Rev")
@Disabled
public class OdomTesting extends LinearOpMode {
    private MechBotNickBot mechBot = new MechBotNickBot();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,mechBot);
    private float[] driveHeading = new float[]{0,0,0};
    @Override
    public void runOpMode() throws InterruptedException {
        mechBot.init(hardwareMap);
        mechBot.colorLeft.enableLed(true);
        mechBot.colorRight.enableLed(true);
        telemetry.addData("Ready to go: ","");
        telemetry.update();
        mechBot.updateOdometry();
        waitForStart();
        telemetry.addData("Starting","");
        telemetry.update();
        while (opModeIsActive()) {
           driveHeading = mechBot.updateOdometry(driveHeading);
           telemetry.addData("","Robot x %.2f  y %.2f  th %.2f ",driveHeading[0],driveHeading[1],(driveHeading[2] * (180.0/Math.PI)));
           telemetry.addData("","Encoder 1 %d Encoder 2 %d Encoder 3 %d Encoder 4 %d",
                   mechBot.one.getCurrentPosition(),mechBot.two.getCurrentPosition(),
                   mechBot.three.getCurrentPosition(),mechBot.four.getCurrentPosition());

            mechBotDriveControls.refreshGamepads(gamepad1,gamepad2);

            if(mechBotDriveControls.isGamepadRefreshed()) {
                if (mechBotDriveControls.joyStickMecnumDrive()) {
                } else if (gamepad1.x) {
                    mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.negX, 1);
                } else if (gamepad1.b) {
                    mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.plusX, 1);
                } else if (gamepad1.y) {
                    mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.plusY, 1);
                } else if (gamepad1.a) {
                    mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.negY, 1);
                }
            }else{
                telemetry.addData("", "Calling joyStickMecnumDrive without updating gamepads");
            }
            /*if(gamepad2.dpad_up){
                mechBot.driveCollecter(-1.0f);
            }else if(gamepad2.dpad_down){
                mechBot.driveCollecter(1.0f);
            }else{
                mechBot.driveCollecter(.0f);

            }*/

           /* if(gamepad2.x){
                mechBot.slideServo.setPower(1);
            }
            else if(gamepad2.b){
                mechBot.slideServo.setPower(-1);
            }
            else{
                mechBot.slideServo.setPower(0);
            }

            if(gamepad2.right_bumper){
                mechBot.kickerServo.setPower(1);
            }else if(gamepad2.left_bumper){
                mechBot.kickerServo.setPower(-1);
            }else{
                mechBot.kickerServo.setPower(0);
            }*/

            if(gamepad2.y){
               mechBot.blockLiftMotor.setPower(1);
            }else if(gamepad2.a){
                mechBot.blockLiftMotor.setPower(-.5);
            }
            else{
                mechBot.blockLiftMotor.setPower(0);
            }

            float[] hsvValues = new float[3];
            float[] hsvValues2 = new float[3];

            Color.RGBToHSV(mechBot.colorLeft.red() * 8, mechBot.colorLeft.green() * 8, mechBot.colorLeft.blue() * 8, hsvValues);
            Color.RGBToHSV(mechBot.colorRight.red() * 8, mechBot.colorRight.green() * 8, mechBot.colorRight.blue() * 8, hsvValues2);
            telemetry.addData("","Sensor One: Red: %d Green %d Blue %d Hue: %f Sat: %f Value:",mechBot.colorLeft.red(),mechBot.colorLeft.green(),mechBot.colorLeft.blue(),
                 hsvValues[0],hsvValues[1],hsvValues[2]);
            telemetry.addData("demo","Sensor Two: Red: %d Green %d Blue %d Hue: %f Sat: %f Value:",mechBot.colorRight.red(),mechBot.colorRight.green(),mechBot.colorRight.blue(),
                   hsvValues2[0],hsvValues2[1],hsvValues2[2]);
            telemetry.update();

        }
    }
}
