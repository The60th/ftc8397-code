package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC Team 8397 on 9/29/2017.
 */
@TeleOp(name="MDarm", group="Rev")
public class mrDriveArmBot extends LinearOpMode {
    private MechBot mechBot = new MechBot(telemetry);
    private  MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,mechBot,1);
    private float[] driveHeading = new float[]{0,0,0};
    @Override
    public void runOpMode() throws InterruptedException {
        mechBot.init(hardwareMap);
        DcMotor claw;
        DcMotor arm;
        claw =  hardwareMap.dcMotor.get("C1");
        arm = hardwareMap.dcMotor.get("A1");
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Ready to go: ","");
        telemetry.update();
        mechBot.updateOdometry();
        waitForStart();
        telemetry.addData("Starting","");
        telemetry.update();
        while (opModeIsActive()) {
            driveHeading = mechBot.updateOdometry(driveHeading);
           /* telemetry.addData("","Robot x %.2f  y %.2f  th %.2f ",driveHeading[0],driveHeading[1],(driveHeading[2] * (180.0/Math.PI)));
            telemetry.addData("","Encoder 1 %d Encoder 2 %d Encoder 3 %d Encoder 4 %d",
                    mechBot.one.getCurrentPosition(),mechBot.two.getCurrentPosition(),
                    mechBot.three.getCurrentPosition(),mechBot.four.getCurrentPosition());*/
            telemetry.update();

            mechBotDriveControls.refreshGamepads(gamepad1,gamepad2);
            if(mechBotDriveControls.isGamepadRefreshed()) {
                if (gamepad1.right_bumper){arm.setPower(.5);}
                else if(gamepad1.left_bumper){arm.setPower(-.5);}
                else{arm.setPower(0);}
                if (gamepad1.dpad_right){claw.setPower(.5);}
                else if (gamepad1.dpad_down){claw.setPower(.1);}
                else if (gamepad1.dpad_left){claw.setPower(-.5);}
                else{claw.setPower(0);}
                //mechBotDriveControls.joyStickMecnumDrive();
                float x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
                float y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
                float a = .0f;
                if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
                    if (gamepad1.left_trigger > gamepad1.right_trigger)
                        a = -gamepad1.left_trigger;
                    else {
                        a = gamepad1.right_trigger;
                    }
                }
                mechBot.setDrivePower(x, y, a);

               /* if (gamepad1.x) {
                    mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.negX, 1);
                } else if (gamepad1.b) {
                    mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.plusX, 1);
                } else if (gamepad1.y) {
                    mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.plusY, 1);
                } else if (gamepad1.a) {
                    mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.negY, 1);
                }*/
            }else{
                telemetry.addData("", "Calling joyStickMecnumDrive without updating gamepads");
                telemetry.update();
            }

        }
    }
}
