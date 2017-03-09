package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by Justi on 2/24/2017.
 */

public class sciFairBot {

    /**
     * Final string constants for controlling the front of the robot.
     */

    public static final String liftFront = "lift";
    static public final String phoneFront = "phone";
    static public final String sweepFront = "sweep";
    static public final String shootFront = "shoot";

    /**
     * Final constant equal to one over the root of two.
     */

    public final float ONE_ROOT_TWO = (float) (1 / Math.sqrt(2));

    /**
     * Final constant equal to the number of ticks per motor rotation with a NeverRest Motor, using a 1-40 Gearbox reduction.
     */
    public final double TICKS_PER_MOTOR_ROTATION = 1120.0; // With a 1 to 40 gearbox.

    /**
     * Final constant equal to current robot gear ratio.
     */
    public final double GEAR_RATIO = 1.0;  //Motor rotations per wheel rotation.

    /**
     * Final constant equal to current robot wheel diameter. Measurement is in inches then convented to centimeters with the value 2.54
     */
    public final double WHEEL_DIAMETER = 4.0 * 2.54;

    /**
     * Final constant equal to our robots encoder ticks per centimeter. Found by taking ticks per motor rotations times gear ratio divided by wheel diameter.
     */
    public final double TICKS_PER_CM = TICKS_PER_MOTOR_ROTATION * GEAR_RATIO / (Math.PI * WHEEL_DIAMETER);

    /**
     * Final constant equal to robot length in centimeters.
     */
    public final double LENGTH = 36.8;

    /**
     * Final constant equal to robot width in centimeters.
     */
    public final double WIDTH = 30.5;

    /**
     * Final constant turn radius which is equal to .5 over root of length times length plus width times width.
     */
    public final double TURN_RADIUS = .5 * (Math.sqrt(LENGTH * LENGTH + WIDTH * WIDTH));

    /**
     * Final constant cosine of beta equal to width plus length over two times root of two times turn radius.
     */
    public final float COS_BETA = (float) ((WIDTH + LENGTH) / (2.0 * Math.sqrt(2.0) * TURN_RADIUS));

    /**
     * Final constant R cosine of beta equal to turn radius times cosine of beta.
     * <p>
     * <p>
     * /**
     * DcMotors one to four used for robot drive wheels.
     */
    public DcMotor one;
    public DcMotor two;
    public DcMotor three;
    public DcMotor four;

    public DcMotor topArm;
    public DcMotor bottomArm;

    public Servo servo;

    /**
     * DcMotor Lift used for robot 80-20 linear slide.
     */

    /**
     * Default Constructor hardwareMap for the class.
     * Setting to null because of no use at current time.
     */
    HardwareMap hardwareMap = null;


    /**
     * Default Constructor OmniBot for the class.
     */
    public sciFairBot() {
    }

    /**
     * Initialize default Hardware interfaces.
     *
     * @param ahwMap passed HardwareMap
     */
    public void init(HardwareMap ahwMap) {

        /**
         * Save passed HardwareMap to local class variable HardwareMap.
         */
        hardwareMap = ahwMap;

        /**
         * Saving all drive wheel motor values to their own string values inside the hardwareMap.
         * Each motor is saved as "M" followed by its number in inger form so Motor One is now "M1".
         */
        one = hardwareMap.dcMotor.get("M1");
        two = hardwareMap.dcMotor.get("M2");
        three = hardwareMap.dcMotor.get("M3");
        four = hardwareMap.dcMotor.get("M4");

        topArm = hardwareMap.dcMotor.get("Top");
        bottomArm = hardwareMap.dcMotor.get("Bottom");

        servo = hardwareMap.servo.get("servo");
        /**
         * Setting the runMode-Direction to reverse for the right launch motors, motor four and motor three.
         * This is because of how all motors are mounted and some need to be in reverse to run correctly.
         */
        four.setDirection(DcMotor.Direction.REVERSE);
        three.setDirection(DcMotor.Direction.REVERSE);

        /**
         * Call our setDriveMode function and pass it the constant DcMotor.RunMode.Run_Using_Encoder so it sets all drive train motors to use encoders.
         *
         * More documentation on the setDriveMode function can be found within it.
         */
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * Set and reconfirm default max ticks per second to a value of 4000.
         * This value controls the max speed at which motors can drive.
         */
        setMaxDriveTicksPerSec(4000);

        /**
         * Call setDrivePower function and pass it the values 0,0,0 and the constant liftFront to set all motor powers to zero, and set default drive mode.
         *
         * More documentation on the setDrivePower function can be found within it.
         *
         */
        setDrivePower(0, 0, 0, liftFront);

        if (OmniBotAutonomous.DEBUG) DbgLog.msg("<Debug> Voltage: voltage = ", getVoltage());
    }


    /**
     * Function to check battery voltage level.
     *
     * @return battery voltage level.
     */
    public double getVoltage() {
        return this.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    /**
     * Sets power for wheels
     *
     * @param px   x velocity
     * @param py   y velocity
     * @param pa   angle
     * @param mode side
     * @return max
     */
    public double setDrivePower(double px, double py, double pa, String mode) {
        //x = +
        //y = -

        double tempHolder;
        switch (mode) {

            //Updated
            //Two versions working check other problems.

            case liftFront:    //Works and does sweeper front currently??
                //Lift front

                //x = y
                //y = x
                tempHolder = py;
                py = -px;
                px = tempHolder;
                break;
            case phoneFront:
                //+x
                //-y

                //y must be - by default?

                //default
                break;
            case shootFront:  //Reveres left and right drive but nothing else.

                px = -px;
                py = -1 * py; //y defualt negtive times - makes it postive

                break;
            case sweepFront:  //Seems to disable all forward drive??
                tempHolder = px;
                py = -tempHolder;
                px = -py;
                break;
        }
        double w1 = -px + py - pa;
        double w2 = px + py - pa;
        double w3 = -px + py + pa;
        double w4 = px + py + pa;
        double max = Math.max(Math.abs(w1), Math.abs(w2));
        max = Math.max(max, Math.abs(w3));
        max = Math.max(max, Math.abs(w4));
        max = Math.max(max, 1.0);
        w1 = w1 / max;
        w2 = w2 / max;
        w3 = w3 / max;
        w4 = w4 / max;
        one.setPower(w1);
        two.setPower(w2);
        three.setPower(w3);
        four.setPower(w4);
        return 1.0 / max;
    }

    /**
     * Sets DC Motor mode
     *
     * @param mode dc motor
     */

    public void setDriveMode(DcMotor.RunMode mode) {
        one.setMode(mode);
        two.setMode(mode);
        three.setMode(mode);
        four.setMode(mode);
    }

    /**
     * @param beh
     */

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior beh) {
        one.setZeroPowerBehavior(beh);
        two.setZeroPowerBehavior(beh);
        three.setZeroPowerBehavior(beh);
        four.setZeroPowerBehavior(beh);
    }

    /**
     * Converts coordinates to motor speed
     *
     * @param vx x velocity
     * @param vy y velocity
     * @param va angle
     * @return wheel powers
     */
    public double setDriveSpeed(double vx, double vy, double va) {
        return setDrivePower(vx * TICKS_PER_CM / (Math.sqrt(2.0) * one.getMaxSpeed()),
                vy * TICKS_PER_CM / (Math.sqrt(2.0) * one.getMaxSpeed()),
                (va * TURN_RADIUS * COS_BETA * TICKS_PER_CM) / one.getMaxSpeed(),
                phoneFront);
    }

    /**
     * @param TicksPerSec
     */

    public void setMaxDriveTicksPerSec(int TicksPerSec) {
        one.setMaxSpeed(TicksPerSec);
        two.setMaxSpeed(TicksPerSec);
        three.setMaxSpeed(TicksPerSec);
        four.setMaxSpeed(TicksPerSec);
    }

    /**
     *
     * @return
     */


}

