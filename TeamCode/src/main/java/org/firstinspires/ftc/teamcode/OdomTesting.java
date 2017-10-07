package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by FTC Team 8397 on 9/29/2017.
 */
@TeleOp(name="OdomTesting", group="Rev")
public class OdomTesting extends LinearOpMode {
    private MechBotSensor mechBot = new MechBotSensor(telemetry);
    private  MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,mechBot);
    private float[] driveHeading = new float[]{0,0,0};
    @Override
    public void runOpMode() throws InterruptedException {
        mechBot.init(hardwareMap);
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
            telemetry.update();

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
                telemetry.update();
            }

        }
    }
}
