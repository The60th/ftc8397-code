package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Jim on 1/17/2017.
 */

@Autonomous(name = "TestLinearMotion", group = "Test Opmodes")
@Disabled
public class TestLinearMotionOpMode extends LinearOpMode {

    OmniBot robot = new OmniBot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        //Initialize robot speeds for forward motion at vy = 20 cm/sec
        float vx = 0.0f;
        float vy = 20.0f;

        //Initialize odometry coordinates
        float[] xyTheta = new float[3];

        //duration of straight line runs
        final double duration = 3000.0;

        //awaitingButtonRelease -- true between the time any button is pressed and when it is released
        boolean awaitingButtonRelease = false;

        //initialize Gyro
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) idle();

        waitForStart();


        //Main loop
        DbgLog.msg("Entering Main Loop");
        while (opModeIsActive()) {

            // if awaiting button release from prior operation, check to see if buttons have been released
            //then continue looping in main loop
            if (awaitingButtonRelease){
                if (!gamepad1.dpad_up && !gamepad1.dpad_right && !gamepad1.start) awaitingButtonRelease = false;
                    continue;
                }

            if (gamepad1.dpad_up){
                awaitingButtonRelease = true;
                vy = vy > 39.9f ? -40.0f : vy + 20.0f;
                DbgLog.msg("Setup: vx = %.0f  vy = %.0f", vx, vy);
                telemetry.addData("Setup", "vx = %.0f  vy = %.0f");
                telemetry.update();
            }
            else if (gamepad1.dpad_right){
                awaitingButtonRelease = true;
                vx = vx > 39.9f ? -40.0f : vx + 20.0f;
                DbgLog.msg("Setup: vx = %.0f  vy = %.0f", vx, vy);
                telemetry.addData("Setup", "vx = %.0f  vy = %.0f");
                telemetry.update();
            }
            else if (gamepad1.start){

                awaitingButtonRelease = true;

                //get gyro reading at beginning of run
                int initialIntegratedZ = robot.sensorGyro.getIntegratedZValue();
                int integratedZ = initialIntegratedZ;

                //initialize odometry readings to x=0, y=0, theta=90 degrees (pi/2 radians)
                xyTheta[0] = 0.0f;
                xyTheta[1] = 0.0f;
                xyTheta[2] = (float)Math.PI/2.0f;

                //set robot wheel ticks to zero and update odometry (to zero out robot.last_Wheel_Ticks)
                robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.updateOdometry();
                robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Logging immediately before straight line run loop
                DbgLog.msg("Entering Straight Line Run Loop");
                //Encoders should all be at zero
                DbgLog.msg("Encoders: %.0f  %.0f  %.0f  %.0f", robot.last_Wheel_Ticks.get(0),
                        robot.last_Wheel_Ticks.get(1), robot.last_Wheel_Ticks.get(2),
                        robot.last_Wheel_Ticks.get(3));
                //x and y should be at zero, theta at 90 degrees
                DbgLog.msg("Odometry: x = %.1f  y = %.1f  Th = %.0f", xyTheta[0], xyTheta[1],
                        xyTheta[2]*180.0/Math.PI);
                //Angle turned should be zero, since we haven't started moving yet
                DbgLog.msg("Gyro: IntZ = %d  AngleTurned = %d", integratedZ, integratedZ-initialIntegratedZ);

                ElapsedTime et = new ElapsedTime();
                robot.setDriveSpeed(vx, vy, 0.0f);
                DbgLog.msg("Motor Powers: %.2f  %.2f  %.2f  %.2f", robot.one.getPower(), robot.two.getPower(),
                        robot.three.getPower(), robot.four.getPower());

                //Straight Line Run Loop
                while (opModeIsActive() && et.milliseconds() < duration){
                    xyTheta = robot.updateOdometry(xyTheta[0], xyTheta[1], xyTheta[2]);
                    integratedZ = robot.sensorGyro.getIntegratedZValue();
                    DbgLog.msg("  Straight Line Run Loop: %.0f ms", et.milliseconds());
                    DbgLog.msg("  Encoders: %.0f  %.0f  %.0f  %.0f", robot.last_Wheel_Ticks.get(0),
                            robot.last_Wheel_Ticks.get(1), robot.last_Wheel_Ticks.get(2),
                            robot.last_Wheel_Ticks.get(3));
                    DbgLog.msg("  Odometry: x = %.1f  y = %.1f  Th = %.0f", xyTheta[0], xyTheta[1],
                            xyTheta[2]*180.0/Math.PI);
                    DbgLog.msg("  Gyro: IntZ = %d  AngleTurned = %d", integratedZ, integratedZ-initialIntegratedZ);
                }

                //Exit Straight Line Run Loop
                DbgLog.msg("End of Straight Line Run Loop:");
                DbgLog.msg("Encoders: %.0f  %.0f  %.0f  %.0f", robot.last_Wheel_Ticks.get(0),
                        robot.last_Wheel_Ticks.get(1), robot.last_Wheel_Ticks.get(2),
                        robot.last_Wheel_Ticks.get(3));
                DbgLog.msg("Odometry: x = %.1f  y = %.1f  Th = %.0f", xyTheta[0], xyTheta[1],
                        xyTheta[2]*180.0/Math.PI);
                DbgLog.msg("Gyro: IntZ = %d  AngleTurned = %d", integratedZ, integratedZ-initialIntegratedZ);

                telemetry.addData("Encoders"," %.0f  %.0f  %.0f  %.0f", robot.last_Wheel_Ticks.get(0),
                robot.last_Wheel_Ticks.get(1), robot.last_Wheel_Ticks.get(2),
                        robot.last_Wheel_Ticks.get(3));
                telemetry.addData("Odometry"," x = %.1f  y = %.1f  Th = %.0f", xyTheta[0], xyTheta[1],
                        xyTheta[2]*180.0/Math.PI);
                telemetry.addData("Gyro"," IntZ = %d  AngleTurned = %d", integratedZ, integratedZ-initialIntegratedZ);
                telemetry.update();
            }
        }

    }


}
