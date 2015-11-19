package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class NxtTeleOpNew extends OpMode {

    // position of the claw servo
    //double clawPosition;

    // amount to change the claw servo position by
   // double clawDelta = 0.01;

    // position of the wrist servo
   // double wristPosition;

    // amount to change the wrist servo position by
   // double wristDelta = 0.01;

    DcMotorController.DeviceMode devMode;
    DcMotorController wheelController;
    DcMotorController WheelRearControler; //New 10/17/15
    DcMotor Tracksright; //motor2 under wheels
    DcMotor Tracksleft; //motor1 under wheels
    DcMotor motorRightWheel;//New 10/17/15 motor 3 under wheels2
    DcMotor motorLeftWheel; //New 10/17/15 motor 4 under wheels

  //  Servo claw;
   // Servo wrist;

    int numOpLoops = 1;

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        Tracksright = hardwareMap.dcMotor.get("motor_2");
        Tracksleft = hardwareMap.dcMotor.get("motor_1");
       // claw = hardwareMap.servo.get("servo_6"); // channel 6
       // wrist = hardwareMap.servo.get("servo_1"); // channel 1
        motorRightWheel = hardwareMap.dcMotor.get("motor_3");
        motorLeftWheel = hardwareMap.dcMotor.get("motor_4");
        wheelController = hardwareMap.dcMotorController.get("wheels");
        WheelRearControler = hardwareMap.dcMotorController.get("tracks");
        devMode = DcMotorController.DeviceMode.WRITE_ONLY;

        Tracksright.setDirection(DcMotor.Direction.REVERSE);    //Controls the direction in what motors the go.
        //Tracksleft.setDirection(DcMotor.Direction.REVERSE);
        //motorLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        motorRightWheel.setDirection(DcMotor.Direction.REVERSE);
        // set the mode
        // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
        Tracksleft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        Tracksright.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS) ;
        motorLeftWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
      //  wristPosition = 0.6;
       // clawPosition = 0.5;
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        // The op mode should only use "write" methods (setPower, setChannelMode, etc) while in
        // WRITE_ONLY mode or SWITCHING_TO_WRITE_MODE
        if (allowedToWrite()) {
    /*
     * Gamepad 1
     *
     * Gamepad 1&2 controls the motors via the left stick and right stick, and it controls the wrist/claw via the a,b,
     * x, y buttons
     */

            if (gamepad1.dpad_left) {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                Tracksleft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                Tracksright.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorRightWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorLeftWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }
            if (gamepad1.dpad_right) {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                Tracksleft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                Tracksright.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                motorRightWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorLeftWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }
            if (gamepad2.dpad_right) {
                Tracksleft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                Tracksright.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorRightWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorLeftWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }
            if (gamepad2.dpad_left) {
                Tracksleft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                Tracksright.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorRightWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorLeftWheel.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }

            // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
            // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
            float throttle2= -gamepad1.left_stick_y; //up Y up x right left
            float direction2 = gamepad1.left_stick_x;
            float throttle = -gamepad1.right_stick_y; //up Y up x right left
            float direction = gamepad1.right_stick_x;
            //float rightleft = throttle - direction;
            //float leftleft = throttle + direction;
            //float throttle = -gamepad1.left_stick_y;
            //float direction = gamepad1.left_stick_x;
            float right = throttle- direction;
            float rightleft = throttle + direction;
            float left = throttle2 + direction2;
            float leftleft = throttle2 - direction2;



            // clip the right/left values so that the values never exceed +/- 1
           rightleft = Range.clip(rightleft, -1, 1);
            leftleft = Range.clip(leftleft, -1, 1);
           right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);


            // write the values to the motors
            Tracksright.setPower(right);
            Tracksleft.setPower(left);
            motorLeftWheel.setPower(leftleft);
            motorRightWheel.setPower(rightleft);
            // update the position of the wrist
           /* if (gamepad1.a) {
                wristPosition -= wristDelta;
            }

            if (gamepad1.y) {
                wristPosition += wristDelta;
            }

            // update the position of the claw
            if (gamepad1.x) {
                clawPosition -= clawDelta;
            }

            if (gamepad1.b) {
                clawPosition += clawDelta;
            }

            // clip the position values so that they never exceed 0..1
            wristPosition = Range.clip(wristPosition, 0, 1);
            clawPosition = Range.clip(clawPosition, 0, 1);

            // write position values to the wrist and claw servo
            wrist.setPosition(wristPosition);
            claw.setPosition(clawPosition);
*/
    /*
     * Gamepad 2
     *
     * Gamepad controls the motors via the right trigger as a throttle, left trigger as reverse, and
     * the left stick for direction. This type of control is sometimes referred to as race car mode.
     */

            // we only want to process gamepad2 if someone is using one of it's analog inputs. If you always
            // want to process gamepad2, remove this check
           /* if (!gamepad2.atRest()) {

                // throttle is taken directly from the right trigger, the right trigger ranges in values from
                // 0 to 1
                throttle = gamepad2.right_trigger;

                // if the left trigger is pressed, go in reverse
                if (gamepad2.left_trigger != 0.0) {
                    throttle = -gamepad2.left_trigger;
                }

                // assign throttle to the left and right motors
                right = throttle;
                left = throttle;

                // now we need to apply steering (direction). The left stick ranges from -1 to 1. If it is
                // negative we want to slow down the left motor. If it is positive we want to slow down the
                // right motor.
                if (gamepad2.left_stick_x < 0) {
                    // negative value, stick is pulled to the left
                    left = left * (1 + gamepad2.left_stick_x);
                }
                if (gamepad2.left_stick_x > 0) {
                    // positive value, stick is pulled to the right
                    right = right * (1 - gamepad2.left_stick_x);
                }

                // write the values to the motor. This will over write any values placed while processing gamepad1
                Tracksright.setPower(right);
                Tracksleft.setPower(left);
            } */
        }

        // To read any values from the NXT controllers, we need to switch into READ_ONLY mode.
        // It takes time for the hardware to switch, so you can't switch modes within one loop of the
        // op mode. Every 17th loop, this op mode switches to READ_ONLY mode, and gets the current power.
        if (numOpLoops % 17 == 0){
            // Note: If you are using the NxtDcMotorController, you need to switch into "read" mode
            // before doing a read, and into "write" mode before doing a write. This is because
            // the NxtDcMotorController is on the I2C interface, and can only do one at a time. If you are
            // using the USBDcMotorController, there is no need to switch, because USB can handle reads
            // and writes without changing modes. The NxtDcMotorControllers start up in "write" mode.
            // This method does nothing on USB devices, but is needed on Nxt devices.
            wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
            WheelRearControler.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);//New 10/17/15
        }

        // Every 17 loops, switch to read mode so we can read data from the NXT device.
        // Only necessary on NXT devices.
        if (wheelController.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {

            // Update the reads after some loops, when the command has successfully propagated through.
            telemetry.addData("Text", "free flow text");
            telemetry.addData("left motor", Tracksleft.getPower());
            telemetry.addData("right motor", Tracksright.getPower());
            telemetry.addData("RunMode: ", Tracksleft.getChannelMode().toString());
            telemetry.addData("RunMode:", motorRightWheel.getPower());
            telemetry.addData("RunMode:", motorLeftWheel.getPower());
            telemetry.addData("RunMode:", motorLeftWheel.getChannelMode().toString());

            // Only needed on Nxt devices, but not on USB devices
            wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
            WheelRearControler.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY); //New 10/17/15

            // Reset the loop
            numOpLoops = 0;
        }

        // Update the current devMode
        devMode = wheelController.getMotorControllerDeviceMode();
        devMode = WheelRearControler.getMotorControllerDeviceMode();//New 10/17/15
        numOpLoops++;
    }

    // If the device is in either of these two modes, the op mode is allowed to write to the HW.
    private boolean allowedToWrite(){
        return (devMode == DcMotorController.DeviceMode.WRITE_ONLY);
    }
}