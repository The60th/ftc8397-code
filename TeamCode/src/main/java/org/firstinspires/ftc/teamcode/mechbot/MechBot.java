package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;


@SuppressWarnings("all")


public class MechBot
{

    protected final boolean LOG_ODOMETRY = true;
    protected final String ODOMETRY_TAG = "ODOMETRY";

    /**
     * Final constant equal to one over the root of two.
     */

    public final  float ONE_ROOT_TWO = (float)(1/Math.sqrt(2));

    public final float ROOT_TWO = (float) Math.sqrt(2);

    /**
     * Final constant equal to the number of ticks per motor rotation with a NeverRest Motor, using a 1-40 Gearbox reduction.
     */
    public final double TICKS_PER_MOTOR_ROTATION_20 = 560; //Ticks per 20.
    public final double TICKS_PER_MOTOR_ROTATION_40 = 1120; //Ticks per 40.



    /**
     * Final constant equal to current robot wheel circumference in cm. Measurement is in inches then convented to centimeters with the value 2.54
     */
    public final double WHEEL_Circumference = 31.5;

    /**
     * Final constant equal to our robots encoder ticks per centimeter. Found by taking ticks per motor rotations times gear ratio divided by wheel diameter.
     */
    public final double TICKS_PER_CM = TICKS_PER_MOTOR_ROTATION_20/WHEEL_Circumference; //Changing to 40 numbers becauase of hardware change.
    public final double MAX_TICKS_PER_SEC = 2500;

    /**
     * Final constant equal to robot length in centimeters.
     */
    //TODO This should be remesured for the newest robot design.
    public final double LENGTH = 33.0;

    /**
     * Final constant equal to robot width in centimeters.
     */
    public final double WIDTH = LENGTH;

    /**
     * Final constant turn radius which is equal to .5 over root of length times length plus width times width.
     */
    public final double TURN_RADIUS = .5*(Math.sqrt(LENGTH*LENGTH+WIDTH*WIDTH));

    public final float R_ROOT_TWO = (float)(TURN_RADIUS*Math.sqrt(2));
    /**
     * Final constant equal to the max value the color sensor can return. This value is returned with extreme colors, or incorrect cable connections and is normally a warning sign.
     */
    public final double MAX_SENSOR_VALUES = 255.0;

    /**
     * Final Matrix to hold the Robot Wheels Transform.
     */
    public final GeneralMatrixF ROBOT_WHEEL_TRANSFORM = new GeneralMatrixF(4,4,
            new float[] {-1, 1, -R_ROOT_TWO,  1
                        , 1, 1, -R_ROOT_TWO, -1
                        ,-1, 1, R_ROOT_TWO, -1
                        , 1, 1, R_ROOT_TWO,  1} );
    public final MatrixF WHEEL_ROBOT_TRANSFORM = ROBOT_WHEEL_TRANSFORM.inverted();

    /**
     * Public Vector to keep track of last returned wheel ticks.
     */
    public VectorF last_Wheel_Ticks = new VectorF(0,0,0,0);

    /**
     * Public float to keep track of last returned gyro theta.
     */
    public float last_Gyro_Theta = 0;

    /**
     * Below are all public variables that are used to store different motor types, Servos, Gyros and Color sensor.
     * Each Object refers to its own item and is named according the what it does.
     */

    /**
     * DcMotors one to four used for robot drive wheels.
     */
    public DcMotor one,two,three,four;

    /**
     * Default Constructor hardwareMap for the class.
     * Setting to null because of no use at current time.
     */
    public HardwareMap hardwareMap = null;

    /**
     * Initialize default Hardware interfaces.
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

        one.setDirection(DcMotor.Direction.REVERSE); //Set up for 20 drive motors.
        two.setDirection(DcMotor.Direction.REVERSE);

        //three.setDirection(DcMotorSimple.Direction.REVERSE);
        //four.setDirection(DcMotorSimple.Direction.REVERSE);
        /**
         * Call our setDriveMode function and pass it the constant DcMotor.RunMode.Run_Using_Encoder so it sets all drive train motors to use encoders.
         *
         * More documentation on the setDriveMode function can be found within it.
         */
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /**
         * Call setDrivePower function and pass it the values 0,0,0 and the constant liftFront to set all motor powers to zero, and set default drive mode.
         *
         * More documentation on the setDrivePower function can be found within it.
         *
         */
        setDrivePower(0,0,0);

    }



    public double setDrivePower(double px, double py, double pa){
        double w1 = -px + py - pa;
        double w2 =  px + py - pa;
        double w3 = -px + py + pa;
        double w4 =  px + py + pa;
        double max = Math.max(Math.abs(w1), Math.abs(w2));
        max = Math.max(max, Math.abs(w3));
        max = Math.max(max, Math.abs(w4));
        max = Math.max(max, 1.0);
        w1 = w1/max;
        w2 = w2/max;
        w3 = w3/max;
        w4 = w4/max;
        one.setPower(w1);
        two.setPower(w2);
        three.setPower(w3);
        four.setPower(w4);
        return 1.0/max;
    }

    /**
     *Sets DC Motor mode
     * @param mode dc motor
     */

    public void setDriveMode(DcMotor.RunMode mode){
        one.setMode(mode);
        two.setMode(mode);
        three.setMode(mode);
        four.setMode(mode);
    }

    /**
     *
     *
     * @param beh
     */

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior beh){
        one.setZeroPowerBehavior(beh);
        two.setZeroPowerBehavior(beh);
        three.setZeroPowerBehavior(beh);
        four.setZeroPowerBehavior(beh);
    }

    /**
     * Converts coordinates to motor speed
     * @param vx x velocity
     * @param vy y velocity
     * @param va angle
     * @return wheel powers
     */
    public double setDriveSpeed(double vx, double vy,double va){
        return setDrivePower(vx * TICKS_PER_CM / MAX_TICKS_PER_SEC,
                             vy * TICKS_PER_CM / MAX_TICKS_PER_SEC,
                            va * TICKS_PER_CM * R_ROOT_TWO /MAX_TICKS_PER_SEC);
    }

    public void updateOdometry(){
        last_Wheel_Ticks.put(0,one.getCurrentPosition());
        last_Wheel_Ticks.put(1,two.getCurrentPosition());
        last_Wheel_Ticks.put(2,three.getCurrentPosition());
        last_Wheel_Ticks.put(3,four.getCurrentPosition());
    }

    /**
     *
     *
     * @param x
     * @param y
     * @param theta
     * @return
     */

    public float[] updateOdometry(float[] lastPos){
        float x = lastPos[0];
        float y = lastPos[1];
        float theta = lastPos[2];
        VectorF curTicks = new VectorF(one.getCurrentPosition(),two.getCurrentPosition(),three.getCurrentPosition(),four.getCurrentPosition());
        VectorF newTicks = curTicks.subtracted(last_Wheel_Ticks);
        VectorF deltaWheelCM = newTicks.multiplied((float)(1.0/TICKS_PER_CM));

        VectorF deltaRobotPos = WHEEL_ROBOT_TRANSFORM.multiplied(deltaWheelCM);
        final float sin = (float)(Math.sin(theta+deltaRobotPos.get(2)/2.0));
        final float cosin = (float)(Math.cos(theta+deltaRobotPos.get(2)/2.0));

        float newX = x+deltaRobotPos.get(0)*sin +deltaRobotPos.get(1)*cosin;
        float newY = y+deltaRobotPos.get(1)*sin - deltaRobotPos.get(0)*cosin;
        float newTheta = theta+deltaRobotPos.get(2);
        last_Wheel_Ticks = curTicks;
        return new float[]{newX,newY,newTheta};
    }

    /**
     *
     * @param x
     * @param y
     * @param theta
     * @param newThetaGyro
     * @return
     */

    //Updated to .2f rather then .1f for logging.
    public float[] updateOdometry(float[] lastPos, float newOdomHeading){
            if (LOG_ODOMETRY) {
                BetaLog.dd(ODOMETRY_TAG, "Entering updateOdometry");
                BetaLog.dd(ODOMETRY_TAG, "lastPos = %.2f %.2f %.2f", lastPos[0], lastPos[1], lastPos[2]);
                BetaLog.dd(ODOMETRY_TAG, "newOdomHeading = %.2f", newOdomHeading);
                BetaLog.dd(ODOMETRY_TAG, "lastWheelTicks = %.0f %.0f %.0f %.0f", last_Wheel_Ticks.get(0),
                        last_Wheel_Ticks.get(1), last_Wheel_Ticks.get(2), last_Wheel_Ticks.get(3));
            }
            float x = lastPos[0];
            float y = lastPos[1];
            float theta = lastPos[2];
            VectorF curTicks = new VectorF(one.getCurrentPosition(), two.getCurrentPosition(), three.getCurrentPosition(), four.getCurrentPosition());
            VectorF newTicks = curTicks.subtracted(last_Wheel_Ticks);
            VectorF deltaWheelCM = newTicks.multiplied((float) (1.0 / TICKS_PER_CM));
            VectorF deltaRobotPos = WHEEL_ROBOT_TRANSFORM.multiplied(deltaWheelCM);
            if (LOG_ODOMETRY) {
                BetaLog.dd(ODOMETRY_TAG, "curTicks = %.0f %.0f %.0f %.0f", curTicks.get(0), curTicks.get(1), curTicks.get(2), curTicks.get(3));
                BetaLog.dd(ODOMETRY_TAG, "newTicks = %.0f %.0f %.0f %.0f", newTicks.get(0), newTicks.get(1), newTicks.get(2), newTicks.get(3));
                BetaLog.dd(ODOMETRY_TAG, "deltaWheelCM = %.2f %.2f %.2f %.2f", deltaWheelCM.get(0), deltaWheelCM.get(1),
                        deltaWheelCM.get(2), deltaWheelCM.get(3));
                BetaLog.dd(ODOMETRY_TAG, "deltaRobotPos = %.2f %.2f %.2f %.2f", deltaRobotPos.get(0), deltaRobotPos.get(1),
                        deltaRobotPos.get(2), deltaRobotPos.get(3));
            }

            float headingChange = (float) VuMarkNavigator.NormalizeAngle(newOdomHeading - theta);
            float averageHeading = theta + headingChange / 2.0f;

            final float sin = (float) (Math.sin(averageHeading));
            final float cosin = (float) (Math.cos(averageHeading));

            float newX = x + deltaRobotPos.get(0) * sin + deltaRobotPos.get(1) * cosin;
            float newY = y + deltaRobotPos.get(1) * sin - deltaRobotPos.get(0) * cosin;

            if (LOG_ODOMETRY)
                BetaLog.dd(ODOMETRY_TAG, "newPos = %.2f %.2f %.2f", newX, newY, newOdomHeading);
            last_Wheel_Ticks = curTicks;
            return new float[]{newX, newY, newOdomHeading};
    }

    // We changed these two methods from adding 90 on the first one to - 90 and the reverse on the second one.
    public float getOdomHeadingFromGyroHeading(float gyroHeading){
        return (float) VuMarkNavigator.NormalizeAngle(gyroHeading - (float)Math.PI / 2.0f);
    }
    public float getGyroHeadingFromOdomHeading(float odomHeading){
        return (float)VuMarkNavigator.NormalizeAngle(odomHeading + (float)Math.PI / 2.0f);
    }
}

