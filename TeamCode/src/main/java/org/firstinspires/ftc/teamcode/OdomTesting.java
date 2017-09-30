package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by FTC Team 8397 on 9/29/2017.
 */
@TeleOp(name="OdomTesting", group="Rev")
public class OdomTesting extends LinearOpMode {
    private MechBotSensor mechBot = new MechBotSensor(telemetry);
    private float[] driveHeading = new float[]{0,0,0};
    private boolean buttonDrive = false;
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
                buttonDrive = false;
                driveHeading = mechBot.updateOdometry(driveHeading);
                telemetry.addData("","Robot x %.2f  y %.2f  th %.2f ",driveHeading[0],driveHeading[1],(driveHeading[2] * (180.0/Math.PI)));
                telemetry.addData("","Encoder 1 %d Encoder 2 %d Encoder 3 %d Encoder 4 %d",
                        mechBot.one.getCurrentPosition(),mechBot.two.getCurrentPosition(),
                        mechBot.three.getCurrentPosition(),mechBot.four.getCurrentPosition());
                telemetry.update();
                double x = 0;
                double y = 0;
                double a = 0;
                x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
                y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
                if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
                    if (gamepad1.left_trigger > gamepad1.right_trigger)
                        a = -gamepad1.left_trigger;
                    else {
                        a = gamepad1.right_trigger;
                    }
                }

                if(gamepad1.x){
                    buttonDrive = true;
                    mechBot.setDrivePower(-(1.0f/4.0f), 0, 0);
                }else if(gamepad1.b){
                    buttonDrive = true;
                    mechBot.setDrivePower(1.0f/4.0f, 0, 0);
                }else if(gamepad1.y){
                    buttonDrive = true;
                    mechBot.setDrivePower(0, (1.0f/4.0f), 0);
                }
                else if(gamepad1.a){
                buttonDrive = true;
                mechBot.setDrivePower(0, -(1.0f/4.0f), 0);
                }
//                telemetry.addData("Drive Mode:","");
//                telemetry.addData("","\n");
//                telemetry.addData("Driving with speeds. " + x/4 + " x " + y/4 + " y " + a/4 + " a ", "");
//                telemetry.update();
                if(!buttonDrive)mechBot.setDrivePower(x/4, y/4, a/4);

        }
    }
}
