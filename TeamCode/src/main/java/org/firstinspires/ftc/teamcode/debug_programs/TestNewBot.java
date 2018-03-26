package org.firstinspires.ftc.teamcode.debug_programs;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.competition_in_work.driver_control.AutoBalance;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;
import org.firstinspires.ftc.teamcode.mechbot.utill.MechBotDriveControls;


/**
 * Created by FTC Team 8397 on 2/20/2018.
 */
@TeleOp(name = "ESR Bot", group = "Tele opmode")
@Disabled

public class TestNewBot extends LoggingLinearOpMode {
    MechBotScranton bot = new MechBotScranton();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1, gamepad2, bot);
    private float[] driveData = new float[6];

    public AutoBalancer autoBalancer = null;
    boolean balancing = false;

    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);
        telemetry.addData("Ready to start TeleOp, waiting for starting button.", "");
        telemetry.update();
        waitForStart();

        bot.setPivotDrive();
        bot.setArmDrive();

        while (opModeIsActive()) {
            if (balancing) {
                if (gamepad1.b) {
                    balancing = true;
                    autoBalancer.update();
                    continue;
                } else {
                    balancing = false;
                    autoBalancer = null;
                    bot.setDrivePower(0, 0, 0);
                }
            } else {
                if (gamepad1.b) {
                    balancing = true;
                    autoBalancer = new AutoBalancer(-30, -120, 0, 60, 20);
                    autoBalancer.start();
                    continue;
                }
            }
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompNewBot(driveData);
            telemetry.addData("Joystick input: ", "X: %.2f Y: %.2f A: %.2f", driveData[0], driveData[1], driveData[2]);
            telemetry.addData("Drive speeds input: ", "X: %.2f Y: %.2f A: %.2f", driveData[3], driveData[4], driveData[5]);
            telemetry.addData("+Left Draw: ", String.format("%.2f", bot.getLeftIntakeCurrentDraw()));
            telemetry.addData("+Right Draw: ", String.format("%.2f", bot.getRightIntakeCurrentDraw()));
            bot.gyroTelemetry(telemetry);
            telemetry.update();
            /*if (gamepad2.right_stick_y > .05){
                bot.rightIntake.setPower(-1);
            }
            else if (gamepad2.right_stick_y < -.05){
                bot.rightIntake.setPower(1);
            }
            else{
                bot.rightIntake.setPower(0);
            }
            if (gamepad2.left_stick_y > .05){
                bot.leftIntake.setPower(-1);
            }
            else if (gamepad2.left_stick_y < -.05){
                bot.leftIntake.setPower(1);
            }
            else{
                bot.leftIntake.setPower(0);
            }*/

            if (gamepad1.right_bumper) {
                bot.rightIntake.setPower(1);
                bot.leftIntake.setPower(1);
            } else if (gamepad1.left_bumper) {
                bot.rightIntake.setPower(-1);
                bot.leftIntake.setPower(-1);
            } else {
                bot.rightIntake.setPower(0);
                bot.leftIntake.setPower(0);
            }

            if (gamepad1.y) {
                bot.setFlipPlateUpwards();
            } else if (gamepad1.a) {
                bot.setFlipPlateDownwards();
            }


            if (gamepad2.a) {
                bot.setGlyphPincherClosed();
            } else if (gamepad2.y) {
                bot.setGlyphPincherMidPos();
            } else if (gamepad2.b) {
                bot.setGlyphPincherStartPos();
            } else if (gamepad2.x) {
                bot.tapGlyphPincher();
            }

            if (gamepad2.right_trigger > .05) {
                bot.setRelicLiftDown();
            } else if (gamepad2.right_bumper) {
                bot.setRelicLiftUp();
            } else {
                bot.setRelicLiftStop();
            }

            if (gamepad2.left_trigger > .05) {
                bot.setRelicClawClosed();
            } else if (gamepad2.left_bumper) {
                bot.setRelicClawOpen();
            }

            if (gamepad2.right_stick_y < -.05) {
                bot.setRelicArmOut();
            } else if (gamepad2.right_stick_y > .05) {
                bot.setRelicArmIn();
            } else {
                bot.setRelicArmStop();
            }

        }


    }

    private class AutoBalancer {
        private float initSpeed;
        private float rollCoeff, pitchCoeff, pitchDerivCoeff, rollDerivCoeff; //Coefficients for control of pitch and roll
        private float initPitch, initRoll;  //Base values of pitch and roll, before starting balance operation
        private ElapsedTime timer;
        private boolean mounted = false;  //Has bot gotten fully onto the stone yet?
        private float prevTimeSec, prevPitchRads, prevRollRads; //Time in seconds from previous update

        public AutoBalancer(float initialSpeed, float rollCoefficient, float rollDerivCoefficient, float pitchCoefficient, float pitchDerivCoefficient) {
            initSpeed = initialSpeed;
            rollCoeff = rollCoefficient;
            pitchCoeff = pitchCoefficient;
            pitchDerivCoeff = pitchDerivCoefficient;
            rollDerivCoeff = rollDerivCoefficient;
        }

        public void start() {
            Orientation angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            initRoll = angles.secondAngle;
            initPitch = angles.thirdAngle;
            timer = new ElapsedTime();
            prevTimeSec = (float) timer.seconds();
            prevPitchRads = initPitch;
            prevRollRads = initRoll;
            bot.setDriveSpeed(0, initSpeed, 0);
        }

        public void update() {
            if (!mounted) {
                int red = bot.colorRight.blue();
                int green = bot.colorRight.green();
                int blue = bot.colorRight.blue();
                float[] hsv = new float[3];
                Color.RGBToHSV(red, green, blue, hsv);
                if (hsv[1] > 0.5) {
                    mounted = true;
                    pidControlIteration();
                }
            } else {
                pidControlIteration();
            }
        }

        private void pidControlIteration() {
            Orientation angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            float roll = angles.secondAngle;
            float pitch = angles.thirdAngle;

            float seconds = (float) timer.seconds();

            float pitchDerivative = (pitch - prevPitchRads) / (seconds - prevTimeSec);
            float vy = pitchCoeff * (pitch - initPitch) + pitchDerivCoeff * pitchDerivative;

            float rollDerivative = (roll - prevRollRads) / (seconds - prevTimeSec);
            float vx = rollCoeff * (roll - initRoll) + rollDerivCoeff + rollDerivative;

            prevTimeSec = seconds;
            prevPitchRads = pitch;
            prevRollRads = roll;
            bot.setDriveSpeed(vx, vy, 0);
        }
    }

}
