package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Important Notice:
 *
 * No robot movement commands are to be used on the START, A or B Buttons on any gamepads.
 * Use of said controls should be REMOVED from all programs, the use of said commands is considered to be unsafe.
 *
 */


@TeleOp(name = "TeleOpDemo: ", group = "TeleOp")
public class newTeleop extends LinearOpMode {
    private OmniBot robot = new OmniBot();
    private VuforiaNav vuforia = new VuforiaNav();
    private float px;
    private float py;
    private float pTheta;
    private float newX = 0;
    private float newY = 0;
    private float newTheta = 0;
    private final float C_PHI = .1f;
    private final float C_X = .1f;
    private String mode = robot.phoneFront;
    private boolean awaitingButtonReleaseServo = false;
    private boolean slowToggle = false;
    private boolean slowToggleControl = false;
    private final float slowModeMod = 4;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        vuforia.activate();
        robot.sensorGyro.calibrate();
        while (opModeIsActive() && robot.sensorGyro.isCalibrating()) {
            telemetry.addData("Gyro Cali", "");
            telemetry.update();
            idle();
        }
        waitForStart();
        robot.updateOdometry();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()) {

            /**
             *
             *  Gamepad One Joy Stick Controls
             *
             */

            px = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
            py = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0;
            pTheta = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0;

            if(slowToggle) {
                robot.setDrivePower(px / slowModeMod, py / slowModeMod, pTheta / slowModeMod, mode);
            }
            else {
                robot.setDrivePower(px, py, pTheta, mode);
            }


            /**
             *
             *  Gamepad One Controls
             *
             */


            if(!gamepad1.guide && slowToggleControl){
                slowToggleControl = !slowToggleControl;
            }
            else if(gamepad1.guide && !slowToggleControl){
                slowToggleControl = true;
                if(!slowToggle){
                    slowToggle = true;
                }
                else if(slowToggle){
                    slowToggle = false;
                }
            }

            //Todo temp place holder testing for a custom turn controller


            if (gamepad1.right_bumper && !(gamepad1.right_trigger > .05)) {
                robot.setSweeper(-1.0);
            } else if (gamepad1.right_trigger > .05 && !gamepad1.right_bumper) {
                robot.setSweeper(1.0);
            } else {
                robot.setSweeper(0.0);
            }

            if (gamepad1.x) {
                teleopBeacon();
                while (opModeIsActive() && gamepad1.x) idle();
            }

            if (gamepad1.dpad_up) {
                mode = OmniBot.phoneFront;
            }
            else if (gamepad1.dpad_down) {
                mode = OmniBot.liftFront;
            }

            /**
             *
             *  Gamepad Two Controls
             *
             */

            if (gamepad2.guide) {
                robot.setServoUp();
            }
            if (gamepad2.left_trigger > .05) {
                robot.setShooter(1.0);
            } else {
                robot.setShooter(0.0);
            }
            if (gamepad2.dpad_down) {
                robot.setBigBallLift(-1);
            } else if (gamepad2.dpad_up) {
                robot.setBigBallLift(1);
            } else {
                robot.setBigBallLift(0);
            }
            if (awaitingButtonReleaseServo && !gamepad2.b && !gamepad2.x) {
                robot.setServos("Reset");
                awaitingButtonReleaseServo = false;
            } else if (gamepad2.b && !awaitingButtonReleaseServo) {
                robot.setServos("Right");
                awaitingButtonReleaseServo = true;
            } else if (gamepad2.x && !awaitingButtonReleaseServo) {
                robot.setServos("Left");
                awaitingButtonReleaseServo = true;
            }
            if (gamepad2.right_trigger > .05) {
                robot.setLaunchServo("Up");
            } else {
                robot.setLaunchServo("Down");
            }


            float pos[] = robot.updateOdometry(newX, newY, newTheta);
            newX = pos[0];
            newY = pos[1];
            newTheta = pos[2];
            if (et.milliseconds() > 500) {
                et.reset();
                telemetry.addData("Gyro Z:", robot.sensorGyro.getIntegratedZValue());
                telemetry.addData("Pos:", "X = %.0f Y = %.0f Theta = %.0f", newX, newY, newTheta * 180.0 / Math.PI);
                telemetry.update();
            }
            idle();
        }
    }

    private void teleopBeacon() throws InterruptedException {
        float v = 20;
        float[] zxPhi = null;
        OpenGLMatrix robotPosition;
        boolean targetFound = false;
        int targetNumber = 21;
        for (int i = 0; i < 4; i++) {
            if (vuforia.isTargetVisible(i)) {
                targetFound = true;
                targetNumber = i;
                break;
            }
        }
        if (!targetFound) return;
        while (opModeIsActive() && gamepad1.x) {
            robotPosition = vuforia.getRobotLocationRelativeToTarget(targetNumber);
            if (robotPosition != null) {
                zxPhi = VuforiaNav.GetZXPH(robotPosition);
            }
            if (zxPhi != null) {
                if (zxPhi[0] <= robot.vuforiaZDistance) {
                    robot.setDriveSpeed(0, 0, 0);
                    return;
                } else {
                    float[] newSpeeds = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v, 0);
                    robot.setDriveSpeed(newSpeeds[0], newSpeeds[1], newSpeeds[2]);
                }
            }
            idle();
        }
        robot.setDriveSpeed(0, 0, 0);
    }

    private float[] getCorrectedSpeeds(float x, float phi, float v, float x0) {
        float phiPrime = VuforiaNav.remapAngle(phi - (float) Math.PI);
        float va = -phiPrime * Math.abs(v) * C_PHI;
        float vx = -C_X * Math.abs(v) * (x - x0) * (float) Math.cos(phiPrime);
        float vy = v + Math.abs(v) * C_X * (x - x0) * (float) Math.sin(phiPrime);
        return new float[]{vx, vy, va};
    }


    public void turnToPosition(float angle, float tolerance, float latency) {
        //Tolerance in degrees latency seconds.

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 135;
        float heading = robot.sensorGyro.getIntegratedZValue();
        float targetHeading = heading + angle;
        float offset = targetHeading - heading;

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            DbgLog.msg("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            robot.setDriveSpeed(0, 0, va * Math.PI / 180.0);
            heading = robot.sensorGyro.getIntegratedZValue();
            offset = targetHeading - heading;
        }
        robot.setDrivePower(0, 0, 0, "");
    }

   /* public void findVuforiaTurn(){
        int j = 0;
        for(int i = 90; i < 130; i = i + 5){
            j = j + 1;
            telemetry.addData("","Gyro current pos: %f : run number %f :",(float)(robot.sensorGyro.getIntegratedZValue()),(float)(i));
            telemetry.update();
            float robotPos = robot.sensorGyro.getIntegratedZValue();

           // if(robotPos+i >= 180) break;

            if(j % 2 == 0) turnToPosition(-(robotPos+((float)(i*2))),3,.3f); else turnToPosition(robotPos+i,3,.3f);
        }
        robot.setDriveSpeed(0,0,0);

    }*/

}
