//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.ftccommon.DbgLog;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//
///**
// * Code written by members of FTC Team Beta 8397.
// * Code to be used for the 2016-2017 FTC competition velocity vortex for driver control period.
// *
// */
//
///**
// * Important Notice:
// *
// * No robot movement commands are to be used on the START, A or B Buttons on any gamepads.
// * Use of said controls should be REMOVED from all programs, the use of said commands is considered to be unsafe.
// *
// */
//
//
///**
// * Notice:
// * This program should be considered locked and as such no changes made without valid reasons.
// * Any changes made should be fully documented with former code left in a comment and reasons for such a change.
// *
// */
//
//@TeleOp(name = "Competition_TeleOp: ", group = "TeleOp_Programs")
//public class newTeleop extends LinearOpMode {
//
//    /**
//     * Declare local versions of inuse classes.
//     */
//    private OmniBot robot = new OmniBot();
//    private VuforiaNav vuforia = new VuforiaNav();
//
//    /**
//     * Global variables px,py,pTheta for position tracking.
//     */
//    private float px;
//    private float py;
//    private float pTheta;
//
//    /**
//     * Global variables to newX,newY,newTheta to track change in X,Y,Theta values.
//     */
//    private float newX = 0;
//    private float newY = 0;
//    private float newTheta = 0;
//
//    /**
//     * Correctional constants, used for correcting movements during Vuforia navigation.
//     */
//    private final float C_PHI = .1f;
//    private final float C_X = .1f;
//
//    /**
//     * Global mode variable to control drive mode.
//     * Defaults to phoneFront.
//     */
//    private String mode = robot.phoneFront;
//
//    /**
//     * Global booleans used for the tracking of button releases, to allow only one input at a time and create a toggle state.
//     */
//    private boolean awaitingButtonReleaseServo = false;
//    private boolean slowToggle = false;
//    private boolean slowToggleControl = false;
//
//    /**
//     * Global Constant used for dividing speed by when in slow mode.
//     */
//    private final float slowModeMod = 4;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        /**
//         * Init and creation of hardware map using the OmniBot class.
//         */
//        robot.init(hardwareMap);
//
//        /**
//         * Activation of Vuforia by using its sub-classes.
//         * Vuforia activation can take up to 100ms
//         */
//        vuforia.activate();
//
//        /**
//         * Activation and calibration of Modern Robotics Gyro sensor by using its builtin methods
//         * Can take up to 10ms to activate and then calibrate.
//         *
//         * At this point the gyro will set the default zero heading to the CURRENT robot heading so robot must be correctly aligned.
//         */
//        robot.sensorGyro.calibrate();
//
//        /**
//         * WhileLoop to prevent other actions from taking place will Gyro is calibrating.
//         *
//         * Imporant Note Here:
//         * All while loops should have "opModeIsActive()" as the default condition along with their required condition.
//         * Example:
//         * While(opModeIsActive() && 2 > 1)
//         * Reason for this precaution is to prevent the app from crashing while if it gets stuck in a while loop.
//         */
//        while (opModeIsActive() && robot.sensorGyro.isCalibrating()) {
//            telemetry.addData("Gyro Cali", "");
//            telemetry.update();
//            idle();
//        }
//
//        /**
//         * Pause at this point till start opMode command is given from the app.
//         */
//        waitForStart();
//
//        /**
//         * Call OmniBot updateOdometry to get starting wheel and other Odemetry values.
//         */
//        robot.updateOdometry();
//
//        /**
//         * Create a global ElaspedTime tracker, to be used for data displaying to lower spam.
//         */
//        ElapsedTime et = new ElapsedTime();
//
//        /**
//         * Main whileLoop the program will stay in till opMode is no longer active.
//         */
//        while (opModeIsActive()) {
//
//            /**
//             *
//             *  Gamepad One Joy Stick Controls
//             *
//             */
//
//            /**
//             * Get values from the joystick when they are greater then our minimum threshold of 0.05.
//             * The left stick on game-pad one controls the main drive wheels using the values px,py.
//             * While the right stick controls rational turn using pTheta
//             *
//             */
//            px = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
//            py = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0;
//            pTheta = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0;
//
//            /**
//             * Check if our slowmode is enabled and if so call our drive power command by divide all values by slowMode constant.
//             * If not call setDrivePower with raw values from the joysticks.
//             *
//             * Information on setDrivePower can be found within its function.
//             */
//            if(slowToggle) {
//                robot.setDrivePower(px / slowModeMod, py / slowModeMod, pTheta / slowModeMod, mode);
//            }
//            else {
//                robot.setDrivePower(px, py, pTheta, mode);
//            }
//
//
//            /**
//             *
//             *  Gamepad One Controls
//             *
//             */
//
//
//            /**
//             * SlowMode toggle.
//             *
//             * Works by toggling a boolean state on first button press then not allowing another input till that state because false again.
//             * The state becomes false when it is true and the button is no longer pressed this works to create a simple toggle on button press.
//             */
//            if(!gamepad1.guide && slowToggleControl){
//                slowToggleControl = !slowToggleControl;
//            }
//            else if(gamepad1.guide && !slowToggleControl){
//                slowToggleControl = true;
//                if(!slowToggle){
//                    slowToggle = true;
//                }
//                else if(slowToggle){
//                    slowToggle = false;
//                }
//            }
//
//
//            /**
//             * Controls for ball pickup(Lift and sweeper)
//             *
//             * Works by running the lift and sweeper motors forward then the right trigger is pressed.
//             * When the right trigger is no longer pressed or the right bumper is pressed the motor will run in reverse.
//             * Otherwise it defaults to an off state.
//             */
//            if (gamepad1.right_bumper && !(gamepad1.right_trigger > .05)) {
//                robot.setSweeper(-1.0);
//            } else if (gamepad1.right_trigger > .05 && !gamepad1.right_bumper) {
//                robot.setSweeper(1.0);
//            } else {
//                robot.setSweeper(0.0);
//            }
//
//            /**
//             * Commands for use of Vuforia Navigation.
//             * When the X button is pressed it will run the teleopBeacon functions which implements parts our Vuforia Navigation Code.
//             * And then waits for the X button to no longer be pressed before stopping.
//             *
//             * Information on the teleopBeacon function can be found within it.
//             */
//            if (gamepad1.x) {
//                teleopBeacon();
//                while (opModeIsActive() && gamepad1.x) idle();
//            }
//
//            /**
//             * Controls to toggle robot front.
//             * This works by waiting the dpad or top hat buttons to be pressed then changing he current mode of the robot.
//             * The mode controls the part of the robot that is considered the front, currently only the sweeper/Lift and phone our supported.
//             *
//             */
//            if (gamepad1.dpad_up) {
//                mode = OmniBot.phoneFront;
//            }
//            else if (gamepad1.dpad_down) {
//                mode = OmniBot.liftFront;
//            }
//
//            /**
//             *
//             *  Gamepad Two Controls
//             *
//             */
//
//            /**
//             * Controls for the holder servo on the linear slide.
//             * When trigger it moves the servo to its "up" or released position allowing the lift to fall down.
//             */
//            if (gamepad2.guide) {
//                robot.setServoUp();
//            }
//
//            /**
//             * Controls for fly wheel launcher.
//             * While the left trigger is press it triggers the wheels till full power.
//             *
//             * Important Note:
//             * While not shown in the code it can take around 500ms for the wheels to reach full speed.
//             */
//            if (gamepad2.left_trigger > .05) {
//                robot.setShooter(1.0);
//            } else {
//                robot.setShooter(0.0);
//            }
//
//            /**
//             * Controls for the linear slide.
//             * On dpad up drive the lift upwards when pressed down drive lift downwards.
//             *
//             */
//            if (gamepad2.dpad_down) {
//                robot.setBigBallLift(1);
//            } else if (gamepad2.dpad_up) {
//                robot.setBigBallLift(-1);
//            } else {
//                robot.setBigBallLift(0);
//            }
//
//            /**
//             * Controls for servo beacon pusher.
//             * Works by using the B and X buttons then pushing the Right/Left servos.
//             */
//            if (awaitingButtonReleaseServo && !gamepad2.b && !gamepad2.x) {
//                robot.setServos("Reset");
//                awaitingButtonReleaseServo = false;
//            } else if (gamepad2.b && !awaitingButtonReleaseServo) {
//                robot.setServos("Right");
//                awaitingButtonReleaseServo = true;
//            } else if (gamepad2.x && !awaitingButtonReleaseServo) {
//                robot.setServos("Left");
//                awaitingButtonReleaseServo = true;
//            }
//
//            /**
//             * Controls for the servo linear actuator used for lifting the ball in to the launcher.
//             * Waits for the right trigger to be pressed then moves the servo in to a position to move the ball upwards.
//             */
//            if (gamepad2.right_trigger > .05) {
//                robot.setLaunchServo("Up");
//            } else {
//                robot.setLaunchServo("Down");
//            }
//
//            /**
//             * Positional data based off of robot Odometry info, compared vs old Data to get change in movement.
//             *
//             */
//            float pos[] = robot.updateOdometry(newX, newY, newTheta);
//            newX = pos[0];
//            newY = pos[1];
//            newTheta = pos[2];
//
//            /**
//             * Wait for et to be greater then 500 ms then display debug data.
//             *
//             */
//            if (et.milliseconds() > 500) {
//                et.reset();
//                telemetry.addData("Gyro Z:", robot.sensorGyro.getIntegratedZValue());
//                telemetry.addData("Pos:", "X = %.0f Y = %.0f Theta = %.0f", newX, newY, newTheta * 180.0 / Math.PI);
//                telemetry.update();
//            }
//            idle();
//        }
//    }
//
//
//    /**
//     * TeleOp beacon function.
//     * Called when the X button is pressed on gamepad2.
//     *
//     * When called the robot switches from a driver control mode to a more Vuforia based autonomous mode.
//     * When first triggered the robot will search for any acceptable Vuforia target and if one is found drive to it.
//     * If none is found it will exit out of the loop.
//     *
//     * @throws InterruptedException
//     */
//    private void teleopBeacon() throws InterruptedException {
//        float v = 20;
//        float[] zxPhi = null;
//        OpenGLMatrix robotPosition;
//        boolean targetFound = false;
//        int targetNumber = 21;
//        for (int i = 0; i < 4; i++) {
//            if (vuforia.isTargetVisible(i)) {
//                targetFound = true;
//                targetNumber = i;
//                break;
//            }
//        }
//        if (!targetFound) return;
//        while (opModeIsActive() && gamepad1.x) {
//            robotPosition = vuforia.getRobotLocationRelativeToTarget(targetNumber);
//            if (robotPosition != null) {
//                zxPhi = VuforiaNav.GetZXPH(robotPosition);
//            }
//            if (zxPhi != null) {
//                if (zxPhi[0] <= robot.vuforiaZDistance) {
//                    robot.setDriveSpeed(0, 0, 0);
//                    return;
//                } else {
//                    float[] newSpeeds = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v, 0);
//                    robot.setDriveSpeed(newSpeeds[0], newSpeeds[1], newSpeeds[2]);
//                }
//            }
//            idle();
//        }
//        robot.setDriveSpeed(0, 0, 0);
//    }
//
//    /**
//     * Correct Speeds Function worked by taking values from Vuforia distance then remapping them in to a speeds used to move to that target.
//     * @param x Passed Vuforia X Distance
//     * @param phi Passed Vuforia phi Distance
//     * @param v Passed Vuforia V Distance
//     * @param x0 Passed Vuforia X0 Distance
//     * @return
//     */
//    private float[] getCorrectedSpeeds(float x, float phi, float v, float x0) {
//        float phiPrime = VuforiaNav.remapAngle(phi - (float) Math.PI);
//        float va = -phiPrime * Math.abs(v) * C_PHI;
//        float vx = -C_X * Math.abs(v) * (x - x0) * (float) Math.cos(phiPrime);
//        float vy = v + Math.abs(v) * C_X * (x - x0) * (float) Math.sin(phiPrime);
//        return new float[]{vx, vy, va};
//    }
//
//    /**
//     * Turns the robot to a set position using the gyro sensor.
//     * Currently not in use and thus lacking some documentation.
//     *
//     * @param angle Angle to drive too.
//     * @param tolerance Allowed tolerance in +- degrees
//     * @param latency Sensor feedback Latency in decimal seconds.
//     */
//
//    public void turnToPosition(float angle, float tolerance, float latency) {
//        //Tolerance in degrees latency seconds.
//
//        final float vaMin = 1.5f * tolerance / latency;
//        final float C = 0.75f / latency;
//        final float vaMax = 135;
//        float heading = robot.sensorGyro.getIntegratedZValue();
//        float targetHeading = heading + angle;
//        float offset = targetHeading - heading;
//
//        while (opModeIsActive() && Math.abs(offset) > tolerance) {
//            float absAdjustedOffset = Math.abs(offset) - tolerance;
//            float absVa = vaMin + C * absAdjustedOffset;
//            absVa = Math.min(absVa, vaMax);
//            float va = absVa * Math.signum(offset);
//            DbgLog.msg("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
//            robot.setDriveSpeed(0, 0, va * Math.PI / 180.0);
//            heading = robot.sensorGyro.getIntegratedZValue();
//            offset = targetHeading - heading;
//        }
//        robot.setDrivePower(0, 0, 0, "");
//    }
//
//
//    /**
//     * Important Notice:
//     * Code below here is still work in progress and hopefully finished soon.
//     *
//     */
//   /* public void findVuforiaTurn(){
//        int j = 0;
//        for(int i = 90; i < 130; i = i + 5){
//            j = j + 1;
//            telemetry.addData("","Gyro current pos: %f : run number %f :",(float)(robot.sensorGyro.getIntegratedZValue()),(float)(i));
//            telemetry.update();
//            float robotPos = robot.sensorGyro.getIntegratedZValue();
//
//           // if(robotPos+i >= 180) break;
//
//            if(j % 2 == 0) turnToPosition(-(robotPos+((float)(i*2))),3,.3f); else turnToPosition(robotPos+i,3,.3f);
//        }
//        robot.setDriveSpeed(0,0,0);
//
//    }*/
//
//}
