package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsConstants;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;


@SuppressWarnings("all")


public class OmniBot
{
    //TODO add version control water mark on robot controler phone andor drive station phone.

    /**
     * Temp enum for color sides
     *
     */
    public static enum TARGET_INDEX{Wheels,Tools,Legos,Gears};

    /**
     * Temp array for target sides.
     *
     */
    public static final int[] blueTargets = {0,2};

    public static final int[] redTargets = {3,1};

    /**
     * Final variable for Vuforia stop distance.
     */
    public final double vuforiaZDistance = 10.16;

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

    public final  float ONE_ROOT_TWO = (float)(1/Math.sqrt(2));

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
    public final double WHEEL_DIAMETER = 4.0*2.54;

    /**
     * Final constant equal to our robots encoder ticks per centimeter. Found by taking ticks per motor rotations times gear ratio divided by wheel diameter.
     */
    public final double TICKS_PER_CM = TICKS_PER_MOTOR_ROTATION*GEAR_RATIO/(Math.PI*WHEEL_DIAMETER);

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
    public final double TURN_RADIUS = .5*(Math.sqrt(LENGTH*LENGTH+WIDTH*WIDTH));

    /**
     * Final constant cosine of beta equal to width plus length over two times root of two times turn radius.
     */
    public final float COS_BETA = (float)( (WIDTH+LENGTH)/(2.0*Math.sqrt(2.0)*TURN_RADIUS));

    /**
     * Final constant R cosine of beta equal to turn radius times cosine of beta.
     */
    public final float R_COS_BETA = (float)(TURN_RADIUS*COS_BETA);

    /**
     * Final constant equal to the minimum acceptable value of red.
     */
    public final int MIN_RED = 15;

    /**
     * Final constant equal to the minimum acceptable value of blue.
     */
    public final int MIN_BLUE = 15;

    /**
     * Final constant equal to the max value the color sensor can return. This value is returned with extreme colors, or incorrect cable connections and is normally a warning sign.
     */
    public final int MAX_SENSOR_VALUES = 255;

    /**
     * Final Matrix to hold the Robot Wheels Transform.
     */
    public final GeneralMatrixF ROBOT_WHEEL_TRANSFORM = new GeneralMatrixF(4,4,
            new float[] {-ONE_ROOT_TWO, ONE_ROOT_TWO, -R_COS_BETA,  1
                        , ONE_ROOT_TWO, ONE_ROOT_TWO, -R_COS_BETA, -1
                        ,-ONE_ROOT_TWO, ONE_ROOT_TWO,  R_COS_BETA, -1
                        , ONE_ROOT_TWO, ONE_ROOT_TWO,  R_COS_BETA,  1} );
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
    public DcMotor one;
    public DcMotor two;
    public DcMotor three;
    public DcMotor four;

    /**
     * DcMotor Lift used for robot 80-20 linear slide.
     */
    public DcMotor Lift;

    /**
     * DcMotors LeftLaunch and RightLaunch used for the fly wheel drive to launch balls.
     */
    public DcMotor LeftLaunch;
    public DcMotor RightLaunch;

    /**
     * DcMotor PickUpLift used to run inner ball lift and the chained ball collector.
     */
    public DcMotor PickupLift;

    /**
     * 180-Degree Rotational Servos LeftPusher and RightPusher used to run the linear actuators to push the beacons.
     */
    public Servo LeftPusher;
    public Servo RightPusher;

    /**
     * 180-Degree Rotational Servo ShooterLift used to move the upwards linear actuator to move the ball in to firing position to launch it.
     */
    public Servo ShooterLift;

    /**
     * 180-Degree Rotational Servo SlideHolder used to control a rotating lock that prevents the lift from falling down.
     */
    public Servo SlideHolder;

    /**
     * Modern Robotics Color Sensor used to read the color of the field beacons to tell which color it is.
     */
    public ColorSensor sensorRGB_One; //Right Side

    //ToDO addition of secondary color sensor, on left side of the robot..

    public ColorSensor sensorRGB_TWO; //Left side

    /**
     * Modern Robotics I2c Gyro Sensor used for getting correct rotational readings to control rotation and correct in-correct movements.
     */
    public ModernRoboticsI2cGyro sensorGyro;

    /**
     * Default Constructor hardwareMap for the class.
     * Setting to null because of no use at current time.
     */
    HardwareMap hardwareMap = null;


    /**
     * Default Constructor OmniBot for the class.
     */
    public OmniBot(){
    }

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
         * Below we take the predeclared objects from above and save them to places in the robot hardwareMap.
         * This allows them to be quick referenced by as a string name.
         */

        /**
         * Saving variable Modern Robotics Color Sensor variable to the hardwareMap path "mr" object can now be refrenced in a config path using "mr".
         */
        sensorRGB_One = hardwareMap.colorSensor.get("mr");

        //TODO Placeholder addon for secondary sensor
        sensorRGB_TWO = hardwareMap.colorSensor.get("mr2");
        /**
         * After saving the color sensor to a new object we rewrite I2c address to a custom value.
         */
        sensorRGB_One.setI2cAddress(I2cAddr.create8bit(0x70));

        /**
         * Saving all drive wheel motor values to their own string values inside the hardwareMap.
         * Each motor is saved as "M" followed by its number in inger form so Motor One is now "M1".
         */
        one = hardwareMap.dcMotor.get("M1");
        two = hardwareMap.dcMotor.get("M2");
        three = hardwareMap.dcMotor.get("M3");
        four = hardwareMap.dcMotor.get("M4");
        /**
         * Saving launch motors to their own string values inside the hardwareMap.
         * Motors are named a shorter version of their starting name down to two letters, LeftLaunch, RightLaunch now LL and RL respectively.
         */
        LeftLaunch = hardwareMap.dcMotor.get("LL");
        RightLaunch = hardwareMap.dcMotor.get("RL");

        /**
         * Saving Lift motor to its own string values inside the hardwareMap.
         * Motor is named with a outdated name "SL" which used to reference "ServoLift" but is now a DcMotor
         */
        Lift = hardwareMap.dcMotor.get("SL");

        /**
         * Saving PickupLift motor to its own string values inside the hardwareMap
         * Motor is named with a outdated name "SG" which used to reference "ServoGrabber" but is now a DcMotor
         */
        PickupLift = hardwareMap.dcMotor.get("SG");

        /**
         * Saving SliderHolder servo to its own string value inside the hardwareMap
         * Servo is named as SSH for servo-slide-holder
         */
        SlideHolder = hardwareMap.servo.get("SSH");

        /**
         * Saves LeftPusher and RightPusher to string values inside the hardwareMap
         */
        LeftPusher = hardwareMap.servo.get("SSL");
        RightPusher = hardwareMap.servo.get("SSR");

        /**
         * Saves ShooterLift to a string value inside the hardwareMap
         */
        ShooterLift = hardwareMap.servo.get("SS");

        /**
         * Saves the Modern Robotics Gyro Sensor to a string value inside the hardwareMap
         * Then sets the Gyro default heading to the Cartesian mode for data reading.
         */
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        sensorGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        /**
         * Setting the motors zero power behaviors to a float mode so the motors the run smooth after being set to zero power.
         * A break behavior would lock the motors when set to zero power then run smooth.
         */
        LeftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /**
         * Setting the runMode-Direction to reverse for the right launch motors, motor four and motor three.
         * This is because of how all motors are mounted and some need to be in reverse to run correctly.
         */
        RightLaunch.setDirection(DcMotor.Direction.REVERSE);
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
        setDrivePower(0,0,0,liftFront);

        /**
         * Call the servo SliderHolder and set it to its default Position of .75.
         * The servo will then be held here by the program under stress.
         */
        SlideHolder.setPosition(1);

        /**
         * Call the servo ShooterLift and set it to its default Position of 1.0.
         * The servo will then be held here by the program under stress.
         */
        ShooterLift.setPosition(1);

        /**
         * Call the Modern Robotics Color Sensor, sensorRGB_One and disable its LED.
         *
         * On the color sensor the LED is used to toggle the sensor from a passive mode and a active mode.
         * With the LED in passive mode the sensor will NOT emmit light and only detect and read data from light sources.
         * With the LED in active mode the sensor will emmit light and use this light to reflect any colors on object back at it and detect its color.
         */
        sensorRGB_One.enableLed(false);
        sensorRGB_TWO.enableLed(false);

        if(OmniBotAutonomous.DEBUG) DbgLog.msg("<Debug> Voltage: voltage = ", getVoltage());
    }


    /**
     * Function to check battery voltage level.
     * @return battery voltage level.
     */
    public double getVoltage(){
        return this.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    /**
     * Class based method to control slide holder servo.
     * Calling function will move servo to up/open position causing the lift to drop.
     */

    public void setServoUp(){
        SlideHolder.setPosition(0);
    } //was .75 before 0

    /**
     *
     * @param p Motor power that is passed to the function.
     * Class based method to control the motor running the linear slide lift.
     *
     */

    public void setBigBallLift(double p){
        Lift.setPower(p);
    }

    /**
     *
     * @param pos Position the servo will drive to.
     *
     */

    public void setLaunchServo(String pos){
        //ToDo Force slow mode when in the air.
        //ToDo use a counter to track seconds of lift and sub seconds of downwards.
        //ToDo Find formula for total time, if greater then threshhold cut all commands and force slow mode.
        if(pos.equals("Up")){
            ShooterLift.setPosition(0);

        }
        else if(pos.equals("Down")){
            ShooterLift.setPosition(1);
        }
    }


    public void setSweeper(Double p){
        PickupLift.setPower(p);
    }

    /**
     *
     * @param p Power that the motors will be set too.
     */

    public void setShooter(Double p){
        LeftLaunch.setPower(p);
        RightLaunch.setPower(p);
    }

    /**
     *
     * @param side chooses which servo to activate
     */

    public void setServos(String side){
            if (side.equals("Left")) {
                LeftPusher.setPosition(1);
            }
            else if (side.equals("Right")) {
                RightPusher.setPosition(1);
            }
            else if(side.equals("Reset")){
                LeftPusher.setPosition(0);
                RightPusher.setPosition(0);
            }
        }

    /**
     *Sets power for wheels
     * @param px x velocity
     * @param py y velocity
     * @param pa angle
     * @param mode side
     * @return max
     */
    public double setDrivePower(double px, double py, double pa, String mode){
        //x = +
        //y = -

        double tempHolder;
        switch (mode){

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
                py = -1*py; //y defualt negtive times - makes it postive

                break;
            case sweepFront:  //Seems to disable all forward drive??
                tempHolder = px;
                py = -tempHolder;
                px = -py;
                break;
        }
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
        return setDrivePower(vx * TICKS_PER_CM / (Math.sqrt(2.0)*one.getMaxSpeed()),
                             vy * TICKS_PER_CM / (Math.sqrt(2.0)*one.getMaxSpeed()),
                            (va * TURN_RADIUS*COS_BETA*TICKS_PER_CM)/one.getMaxSpeed(),
                            phoneFront);
    }

    /**
     *
     *
     * @param TicksPerSec
     */

    public void setMaxDriveTicksPerSec(int TicksPerSec){
        one.setMaxSpeed(TicksPerSec);
        two.setMaxSpeed(TicksPerSec);
        three.setMaxSpeed(TicksPerSec);
        four.setMaxSpeed(TicksPerSec);
    }

    /**
     *
     * @return
     */

    public int[] getMaxDriveTicksPerSec (){
        int[] returnVal = new int[4];
        returnVal[0] = one.getMaxSpeed();
        returnVal[1] = two.getMaxSpeed();
        returnVal[2] = three.getMaxSpeed();
        returnVal[3] = four.getMaxSpeed();
        return returnVal;
    }

    /**
     *Determines if beacon is red
     * @return true or false if beacon is red
     */

    //ToDo add data for left side of beacon.
    public boolean isRightBeaconRed(){
        float redRight = sensorRGB_One.red();
        float blueRight = sensorRGB_One.blue();
        float alphaRight = sensorRGB_One.alpha();
        float greenRight = sensorRGB_One.green();
        return(redRight >= MIN_RED && !(redRight == MAX_SENSOR_VALUES && blueRight == MAX_SENSOR_VALUES && alphaRight == MAX_SENSOR_VALUES && greenRight == MAX_SENSOR_VALUES));
    }

    /**
     *Determines if beacon is blue
     * @return true or false if beacon is blue
     */

    public boolean isRightBeaconBlue(){
        float redRight = sensorRGB_One.red();
        float blueRight = sensorRGB_One.blue();
        float alphaRight = sensorRGB_One.alpha();
        float greenRight = sensorRGB_One.green();
        return(blueRight >= MIN_BLUE && !(redRight == MAX_SENSOR_VALUES && blueRight == MAX_SENSOR_VALUES && alphaRight == MAX_SENSOR_VALUES && greenRight == MAX_SENSOR_VALUES));
    }

    //ToDo Wip
    /**
     *
     * @return
     */
    public boolean isLeftBeaconBlue(){
        float redLeft = sensorRGB_TWO.red();
        float blueLeft = sensorRGB_TWO.blue();
        float alphaLeft = sensorRGB_TWO.alpha();
        float greenLeft = sensorRGB_TWO.green();
        return(blueLeft >= MIN_BLUE && !(redLeft == MAX_SENSOR_VALUES && blueLeft == MAX_SENSOR_VALUES && alphaLeft == MAX_SENSOR_VALUES && greenLeft == MAX_SENSOR_VALUES));
    }

    //ToDo Wip
    /**
     *
     * @return
     */
    public boolean isLeftBeaconRed(){
        float redLeft = sensorRGB_TWO.red();
        float blueLeft = sensorRGB_TWO.blue();
        float alphaLeft = sensorRGB_TWO.alpha();
        float greenLeft = sensorRGB_TWO.green();
        return(redLeft >= MIN_RED && !(redLeft == MAX_SENSOR_VALUES && blueLeft == MAX_SENSOR_VALUES && alphaLeft == MAX_SENSOR_VALUES && greenLeft == MAX_SENSOR_VALUES));
    }

    //ToDo wip: Function for double beacon color check.
    public String findColor(String color) {
        //Colors inside the float in RGB order
        float[] colorsRight = {sensorRGB_One.red(),sensorRGB_One.green(),sensorRGB_One.blue()};
        float[] colorsLeft = {sensorRGB_TWO.red(),sensorRGB_TWO.green(),sensorRGB_TWO.blue()};
        switch (color) {
            case "Red":
                if((colorsRight[0] > MIN_RED && colorsRight[2] < MIN_BLUE) && !(colorsRight[0] == MAX_SENSOR_VALUES&&colorsRight[1] == MAX_SENSOR_VALUES&&colorsRight[2] == MAX_SENSOR_VALUES)){
                    return "Right";
                }
                else if((colorsLeft[0] > MIN_RED && colorsLeft[2] < MIN_BLUE) && !(colorsRight[0] == MAX_SENSOR_VALUES&&colorsRight[1] == MAX_SENSOR_VALUES&&colorsRight[2] == MAX_SENSOR_VALUES)){
                    return "Left";
                }
                break;
            case "Blue":
                if((colorsRight[0] > MIN_BLUE && colorsRight[2] < MIN_RED)&&!(colorsLeft[0] == MAX_SENSOR_VALUES&&colorsLeft[1] == MAX_SENSOR_VALUES&&colorsLeft[2] == MAX_SENSOR_VALUES)){
                    return "Right";
                }
                else if((colorsLeft[0] > MIN_BLUE && colorsLeft[2] < MIN_RED)&&!(colorsLeft[0] == MAX_SENSOR_VALUES&&colorsLeft[1] == MAX_SENSOR_VALUES&&colorsLeft[2] == MAX_SENSOR_VALUES)){
                    return "Left";
                }
                break;
        }
        return "";

    }

    /**
     *Gets color values from color sensor
     * @return float array of color values
     */
    public float[] getColorValues(){
        float[] colorValues = {sensorRGB_One.red(),sensorRGB_One.green(),sensorRGB_One.blue()};
       return colorValues;
    }

    /**
     *
     * @param
     *
     *
     */
    public void updateOdometry(){
        last_Wheel_Ticks.put(0,one.getCurrentPosition());
        last_Wheel_Ticks.put(1,two.getCurrentPosition());
        last_Wheel_Ticks.put(2,three.getCurrentPosition());
        last_Wheel_Ticks.put(3,four.getCurrentPosition());
        //last_Gyro_Theta = (float)((double)-sensorGyro.getIntegratedZValue()*(Math.PI/180.0));
    }

    /**
     *
     *
     * @param x
     * @param y
     * @param theta
     * @return
     */

    public float[] updateOdometry(float x, float y, float theta){
        VectorF curTicks = new VectorF(one.getCurrentPosition(),two.getCurrentPosition(),three.getCurrentPosition(),four.getCurrentPosition());
        VectorF newTicks = curTicks.subtracted(last_Wheel_Ticks);
        VectorF deltaWheelCM = newTicks.multiplied((float)(1.0f/TICKS_PER_CM));

        VectorF deltaRobotPos = WHEEL_ROBOT_TRANSFORM.multiplied(deltaWheelCM);
        final float sin = (float)(Math.sin(theta+deltaRobotPos.get(2)/2.0));
        final float cosin = (float)(Math.cos(theta+deltaRobotPos.get(2)/2.0));

        float newX = x+deltaRobotPos.get(0)*sin +deltaRobotPos.get(1)*cosin;
        float newY = y+deltaRobotPos.get(1)*sin - deltaRobotPos.get(0)*cosin;
        float newTheta = theta+deltaRobotPos.get(2);
        last_Wheel_Ticks = curTicks;
       // last_Gyro_Theta = (float)((double)-sensorGyro.getIntegratedZValue()*(Math.PI/180.0));
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
    public float[] updateOdometry(float x,float y,float theta,float newThetaGyro){
        return null;
    }

    /*public float[] updateOdometryGyro(float x,float y,float theta){
        VectorF curTicks = new VectorF(one.getCurrentPosition(),two.getCurrentPosition(),three.getCurrentPosition(),four.getCurrentPosition());
        VectorF newTicks = curTicks.subtracted(last_Wheel_Ticks);
        VectorF deltaWheelCM = newTicks.multiplied((float)(1.0f/TICKS_PER_CM));

        VectorF deltaRobotPos = WHEEL_ROBOT_TRANSFORM.multiplied(deltaWheelCM);
        float newGyroTheta = (float)((double)-sensorGyro.getIntegratedZValue()*(Math.PI/180.0));
        float deltaGyroTheta = newGyroTheta-last_Gyro_Theta;

        final float sin = (float)(Math.sin(theta+deltaGyroTheta/2.0));
        final float cosin = (float)(Math.cos(theta+deltaGyroTheta/2.0));

        float newX = x+deltaRobotPos.get(0)*sin +deltaRobotPos.get(1)*cosin;
        float newY = y+deltaRobotPos.get(1)*sin - deltaRobotPos.get(0)*cosin;
        float newTheta = theta+newGyroTheta;
        last_Wheel_Ticks = newTicks;
        last_Gyro_Theta = newGyroTheta;

        return new float[]{newX,newY,newTheta};
    }*/

    public OmniBotAutonomous.BeaconColor getLeftColor(){
        //sensorRGB_TWO
        int red = sensorRGB_TWO.red();
        int blue = sensorRGB_TWO.blue();

        if(OmniBotAutonomous.DEBUG) DbgLog.msg("<Debug-Color> getLeftColor-Color values red %d blue %d",red,blue);
        if(red == MAX_SENSOR_VALUES || blue == MAX_SENSOR_VALUES) return OmniBotAutonomous.BeaconColor.Unknown;
        else if(red>MIN_RED && red > (blue*1.5)) return OmniBotAutonomous.BeaconColor.Red;
        else if(blue>MIN_BLUE && blue > (red*1.5)) return OmniBotAutonomous.BeaconColor.Blue;
        else return OmniBotAutonomous.BeaconColor.Unknown;
    }
    public OmniBotAutonomous.BeaconColor getRightColor(){
        //sensorRGB_One
        int red = sensorRGB_One.red();
        int blue = sensorRGB_One.blue();

        if(OmniBotAutonomous.DEBUG) DbgLog.msg("<Debug-Color> getRightColor-Color values red %d blue %d",red,blue);
        if(red == MAX_SENSOR_VALUES || blue == MAX_SENSOR_VALUES) return OmniBotAutonomous.BeaconColor.Unknown;
        else if(red>MIN_RED && red > (blue*1.5)) return OmniBotAutonomous.BeaconColor.Red;
        else if(blue>MIN_BLUE && blue > (red*1.5)) return OmniBotAutonomous.BeaconColor.Blue;
        else return OmniBotAutonomous.BeaconColor.Unknown;
    }
}

