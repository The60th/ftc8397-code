package org.firstinspires.ftc.teamcode.competition_in_work.driver_control;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;
import org.firstinspires.ftc.teamcode.mechbot.utill.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 3/14/2018.
 */
@TeleOp(name = "TeleOp ", group = "TeleOp")
public class WorldsTeleOP extends LoggingLinearOpMode {
    public int flipPlateUpticks = 580;
    public int flipPlateDownticks = 0;
    int tickOffSet = 0;

    ElapsedTime et = new ElapsedTime();
    MechBotScranton bot = new MechBotScranton();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1, gamepad2, bot);
    private float[] driveData = new float[6];

    double pos = 0.0;
    double UPpos = .98;

    public AutoBalancer autoBalancer = null;
    boolean balancing = false;
    UTILToggle bottomToggle = new UTILToggle();
    boolean kickerStatus = false;
    boolean flipUp = false;
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
                    //autoBalancer = new AutoBalancer(-30, -120, 0, 60, 20);
                    autoBalancer = new AutoBalancer(30,-120,0,+120,+0);
                    autoBalancer.start();
                    continue;
                }
            }
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompNewBot(driveData);
           // telemetry.addData("Joystick input: ", "X: %.2f Y: %.2f A: %.2f", driveData[0], driveData[1], driveData[2]);
           // telemetry.addData("Drive speeds input: ", "X: %.2f Y: %.2f A: %.2f", driveData[3], driveData[4], driveData[5]);
           // telemetry.addData("+Left Draw: ", String.format("%.2f", bot.getLeftIntakeCurrentDraw()));
            //telemetry.addData("+Right Draw: ", String.format("%.2f", bot.getRightIntakeCurrentDraw()));
           // telemetry.addData("Tick Off Set",tickOffSet);
           // telemetry.addData("Glyph color: ", bot.getIntakeColor());
           // telemetry.addData("ODS Distance (CM):", bot.getIntakeDistance());
           // bot.gyroTelemetry(telemetry);
           // telemetry.update();

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

            if(gamepad1.right_stick_y < -.05 && flipUp && et.milliseconds() > 10){
                tickOffSet-=4;
                if(tickOffSet < -125) tickOffSet = -100;
                bot.setFlipPosition(flipPlateUpticks +tickOffSet);
                if(kickerStatus) {
                    bot.setRetractKicker();
                    kickerStatus = false;
                }
                bot.backStop.setPosition(.0);
                et.reset();
            }
            else if(gamepad1.right_stick_y >.05 && flipUp && et.milliseconds() > 10){
                tickOffSet+=4;
                if(tickOffSet > 125) tickOffSet = 100;
                bot.setFlipPosition(flipPlateUpticks +tickOffSet);
                if(kickerStatus) {
                    bot.setRetractKicker();
                    kickerStatus = false;
                }
                bot.backStop.setPosition(.0);
                et.reset();
            }

            if (gamepad1.dpad_up){
                flipUp = false;
                if(kickerStatus) {
                    bot.setRetractKicker();
                    kickerStatus = false;
                }
                bot.backStop.setPosition(0.00);
               // bot.setFlipPosition(bot.flipMotor.getCurrentPosition() + 4);
                bot.flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bot.flipMotor.setPower(.1);
                tickOffSet = 0;
            }
            else if (gamepad1.dpad_down){
                flipUp = false;
                if(kickerStatus) {
                    bot.setRetractKicker();
                    kickerStatus = false;
                }
                bot.backStop.setPosition(0.00);
                //bot.setFlipPosition(bot.flipMotor.getCurrentPosition() - 4);
                bot.flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bot.flipMotor.setPower(-.1);
                tickOffSet = 0;
            }
            else if (gamepad1.dpad_right){
                bot.flipMotor.setPower(0);
                int upDownDifference = flipPlateUpticks - flipPlateDownticks;
                flipPlateDownticks = bot.flipMotor.getCurrentPosition();
                flipPlateUpticks = flipPlateDownticks + upDownDifference;
                tickOffSet = 0;
                flipUp = false;
            }

           /* if(gamepad1.dpad_up){
                tickOffSet+=10;
                bot.setFlipPosition(flipPlateUpticks+tickOffSet);
                if(kickerStatus) {
                    bot.setRetractKicker();
                    kickerStatus = false;
                }
                bot.backStop.setPosition(.0);
            }else if(gamepad1.dpad_down){
                tickOffSet-=10;
                bot.setFlipPosition(flipPlateUpticks+tickOffSet);
                if(kickerStatus) {
                    bot.setRetractKicker();
                    kickerStatus = false;
                }
                bot.backStop.setPosition(.0);
            }
*/
            if(gamepad1.y) {
                flipUp = true;
                if(kickerStatus) {
                    bot.setRetractKicker();
                    kickerStatus = false;
                }
                bot.backStop.setPosition(0.00);
                bot.setFlipPosition(flipPlateUpticks);
                tickOffSet = 0;
            }
            else if(gamepad1.a){
                flipUp = false;
                if(kickerStatus) {
                    bot.setRetractKicker();
                    kickerStatus = false;
                }
                bot.backStop.setPosition(0.280);
                bot.setFlipPosition(flipPlateDownticks);
                tickOffSet = 0;
            }

            if (gamepad2.a) {
                bot.setGlyphPincherClosed();
            } else if (gamepad2.y) {
                bot.setGlyphPincherMidPos();
            } else if (gamepad2.b) {
                bot.setGlyphPincherStartPos();
            }

            if (bottomToggle.status(gamepad2.x) == UTILToggle.Status.COMPLETE) {
                if(!kickerStatus) {
                    bot.setKickGlyph();
                }
                else if(kickerStatus) {
                    bot.setRetractKicker();
                }
                kickerStatus = !kickerStatus;
            }


            if (gamepad2.right_trigger > .15) {
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
            bot.pivot.setPosition(pos);
            bot.arm.setPosition(UPpos);
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
                int red = bot.backColorRight.blue();
                int green = bot.backColorRight.green();
                int blue = bot.backColorRight.blue();
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
//
//        private void CryptoNav(){
//            boolean yMode = false;
//            boolean xMode = false;
//            robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//            CameraDevice.getInstance().setFlashTorchMode(true);
//
//            BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuMarkNavigator.getFrameQueue();
//
//            VuMarkNavigator.clearFrameQueue(frameQueue);
//
//            BetaLog.dd(TAG, "Cleared Frame Queue");
//
//            boolean cryptoNavInitialized = false;
//            while (opModeIsActive()){
//                //Try to get an image; if image not yet available, loop and try again.
//                if (!VuMarkNavigator.getRGB565Array(frameQueue, rawImgWidth, rawImgHeight, imageBytes)) continue;
//
//                BetaLog.dd(TAG, "Got CryptoNav Initialization Image");
//
//                //Try to initialize the rails. We're assuming for this demonstration that the left-most
//                //rail is visible to the camera before initializeRails is called.
//                if(!CryptoNav.initializeRails(imageBytes, CryptoNav.Side.LEFT)) break;
//                BetaLog.dd(TAG, "CryptoNav Initialization Succeeded");
//                cryptoNavInitialized = true;
//                break;
//            }
//
//            if (cryptoNavInitialized) {
//                BetaLog.dd(TAG, "CryptoNav Initialization Succeeded");
//                telemetry.addData(TAG,"CryptoNav Initialization Succeeded.");
//                telemetry.update();
//            }
//            else {
//                BetaLog.dd(TAG, "CryptoNav Initialization Failed.");
//                telemetry.addData(TAG,"CryptoNav Initialization Failed.");
//                telemetry.update();
//                while (opModeIsActive()){
//                    if(!gamepad1.x) break;
//                    continue;
//                }
//            }
//            boolean whileControl = false;
//            BetaLog.dd(TAG, "Entering CryptoNav navigation loop");
//            while (opModeIsActive()) {
//
//                if(gamepad1.x) {
//                    whileControl = true;
//                }
//                //SEVENTH: Enter the actual navigation loop. In practice, each iteration of this loop would include adjustments
//                //of motor powers, just as we did with Vuforia navigation.
//                if (gamepad1.y && !yMode) {
//                    telemetry.addData(TAG, "Gamepad1:");
//                    telemetry.addData(TAG, "Y Button pressed entering secondary mode.");
//                    telemetry.addData(TAG, "Press A to exit.");
//                    yMode = true;
//                }
//                else if (gamepad1.a && yMode) {
//                    telemetry.addData(TAG, "Gamepad1:");
//                    telemetry.addData(TAG, "A button pressed disabling secondary mode.");
//                    telemetry.addData(TAG, "Press Y to enter secondary mode.");
//                    yMode = false;
//                }
//
//                if (gamepad2.y && !xMode) {
//                    telemetry.addData(TAG, "Gamepad2:");
//                    telemetry.addData(TAG, "Y Button pressed entering secondary mode.");
//                    telemetry.addData(TAG, "Press A to exit.");
//                    xMode = true;
//                }
//                else if (gamepad2.a && xMode) {
//                    telemetry.addData(TAG, "Gamepad2:");
//                    telemetry.addData(TAG, "A button pressed disabling secondary mode.");
//                    telemetry.addData(TAG, "Press Y to enter secondary mode.");
//                    xMode = false;
//                }
//                telemetry.update();
//
//
//                while (opModeIsActive() && whileControl) {
//
//                    //Get a new image; if no image is a available, keep on trying.
//                    if (!VuMarkNavigator.getRGB565Array(frameQueue, rawImgWidth, rawImgHeight, imageBytes))
//                        continue;
//
//                    BetaLog.dd(TAG, "Got CrytoNav navigation image");
//
//                    //From the image, use the CryptoNav.updateLocationZX method to obtain new z,x coordinates.
//                    //Note that this method does not GIVE us the phiPrime heading value. Instead, we need to provide the
//                    //method with the phiPrime value. In practice, we would obtain this from the GYRO. For this
//                    //demonstration, it will be assumed that phiPrime is 0 (just keep the phone pointed directly toward the
//                    //wall.
//                    float gyroHeading = bot.getHeadingRadians();
//                    float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);
//                    float cameraHeading = bot.getCameraHeadingFromGyroHeading(gyroHeading);
//
//                    float[] zx = CryptoNav.updateLocationZX(imageBytes, cameraHeading);
//                    float[] robotZX = new float[3];
//                    //Note: if navigation fails, that's usually because only zero or one rails is currently visible.
//                    //The updateLocationZX method needs to "see" at least two rails. No problem; once two rails become
//                    //visible again, we should start getting valid locations again.
//                    if (zx == null) {
//                        BetaLog.dd(TAG, "Crypto Navigation Failed");
//                        telemetry.addData("Crypto Navigation Failed", "");
//                        //robotZXPhi = bot.updateOdometry(robotZXPhi,odomHeading);
//                        while (!gamepad1.b) {
//                            telemetry.addData("Crypto Navigation Failed", "");
//                            telemetry.addData("Press gamepad1 B to restart.", "");
//                            telemetry.update();
//                            continue;
//                        }
//                    } else {
//                        BetaLog.dd(TAG, "Crypto Navigation Succeeded");
//                        BetaLog.dd(TAG, "Camera Coords: z = %.1f  x = %.1f", zx[0], zx[1]);
//                        telemetry.addData("Crypto Navigation Succeeded, Camera:", " z=%.1f x=%.1f", zx[0], zx[1]);
//                        robotZX = bot.getRobotZXfromCameraZX(zx, gyroHeading);
//                        setOdometry(robotZX[0], robotZX[1], odomHeading);
//
//                    }
//
//                    BetaLog.dd("Robot coords:", " z=%.1f  x=%.1f  phi=%.1f", robotZX[0], robotZX[1],
//                            odomHeading * 180.0 / Math.PI);
//                    telemetry.addData("Robot coords:", " z=%.1f x=%.1f phi=%.1f", robotZX[0], robotZX[1],
//                            odomHeading * 180.0 / Math.PI);
//                    if (gamepad1.b) whileControl = false;
//
//
//                    BetaLog.dd(TAG,"My mode is: ", yMode + ". My mode is: " + xMode);
//                    //if(robotZXPhi[0] < 41) break;
//                    final float vNom = 5.0f;
//                    final float coeff = 1.0f;
//                    float vX = -vNom * (float) Math.cos(cameraHeading) - coeff * robotZX[1] * (float) Math.sin(cameraHeading);
//                    float vY = vNom * (float) Math.sin(cameraHeading) - coeff * robotZX[1] * (float) Math.cos(cameraHeading);
//                    float vA = -HEADING_CORECTION_FACTOR * (float) VuMarkNavigator.NormalizeAngle(gyroHeading + Math.PI / 2.0f);
//                    if (yMode) {
//                        bot.setDriveSpeed(0, vY, 0);
//                        BetaLog.dd("Robot speed values:", " vX=%.1f  vY=%.1f  vA=%.1f", 0.0f, vY, 0.0f);
//                    } else if(xMode) {
//                        bot.setDriveSpeed(vX, 0, 0);
//                        BetaLog.dd("Robot speed values:", " vX=%.1f  vY=%.1f  vA=%.1f", vX, 0.0f, 0.0f);
//                    }else{
//                        bot.setDriveSpeed(vX,vY,0);
//                        BetaLog.dd("Robot speed set:", " vX=%.1f  vY=%.1f  vA=%.1f", vX, vY, 0.0f);
//                    }
//                    BetaLog.dd("Robot speed calculated:", " vX=%.1f  vY=%.1f  vA=%.1f", vX, vY, vA);
//                    telemetry.update();
//                }
//
//                bot.setDriveSpeed(0, 0, 0);
//                CameraDevice.getInstance().setFlashTorchMode(false);
//            }
//        }
//
//
//    protected void setOdometry(float z, float x, float odomHeading){
//        robotZXPhi = new float[] {z, x, odomHeading};
//        bot.updateOdometry();
//    }
//
//    protected void setOdometry(float z, float x){
//        robotZXPhi = new float[] {z, x, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//        bot.updateOdometry();
//    }

}
