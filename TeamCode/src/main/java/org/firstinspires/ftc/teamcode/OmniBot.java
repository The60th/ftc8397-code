package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class OmniBot
{
    //Robot constants
    public final  float ONE_ROOT_TWO = (float)(1/Math.sqrt(2));
    public final double TICKS_PER_MOTOR_ROTATION = 1120.0; // With a 1 to 40 gearbox.
    public final double GEAR_RATIO = 1.0;  //Motor rotations per wheel rotation.
    public final double WHEEL_DIAMETER = 4.0*2.54;  //4in converted to cm.
    public final double TICKS_PER_CM = TICKS_PER_MOTOR_ROTATION*GEAR_RATIO/(Math.PI*WHEEL_DIAMETER);
    public final double LENGTH = 36.8; //cm
    public final double WIDTH = 30.5;  //cm
    public final double TURN_RADIUS = .5*(Math.sqrt(LENGTH*LENGTH+WIDTH*WIDTH));
    public final float COS_BETA = (float)( (WIDTH+LENGTH)/(2.0*Math.sqrt(2.0)*TURN_RADIUS));
    public final float R_COS_BETA = (float)(TURN_RADIUS*COS_BETA);
    public final double MIN_RED = 20.0;
    public final double MIN_BLUE = 20.0;
    public final double MAX_SENSOR_VALUES = 255.0;
    public final GeneralMatrixF ROBOT_WHEEL_TRANSFORM = new GeneralMatrixF(4,4,
            new float[] {-ONE_ROOT_TWO, ONE_ROOT_TWO, -R_COS_BETA,  1
                        , ONE_ROOT_TWO, ONE_ROOT_TWO, -R_COS_BETA, -1
                        ,-ONE_ROOT_TWO, ONE_ROOT_TWO,  R_COS_BETA, -1
                        , ONE_ROOT_TWO, ONE_ROOT_TWO,  R_COS_BETA,  1} );
    public final MatrixF WHEEL_ROBOT_TRANSFORM = ROBOT_WHEEL_TRANSFORM.inverted();
    public VectorF last_Wheel_Ticks = new VectorF(0,0,0,0);
    public float last_Gyro_Theta = 0;

    /* Public OpMode members. */
    public DcMotor one;
    public DcMotor two;
    public DcMotor three;
    public DcMotor four;
    public DcMotor Lift;
    public DcMotor Grabber;
    public DcMotor LeftLaunch;
    public DcMotor RightLaunch;
    public Servo LeftPusher;
    public Servo RightPusher;
    public ColorSensor sensorRGB_One;
    public ModernRoboticsI2cGyro sensorGyro;
    //public ColorSensor sensorRGB2;

    /* local OpMode members. */
    HardwareMap hardwareMap = null;

    /* Constructor */
    public OmniBot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        sensorRGB_One = hardwareMap.colorSensor.get("mr"); //Added color sensors for running the robot
        //sensorRGB2 = hardwareMap.colorSensor.get("mr2");
        sensorRGB_One.setI2cAddress(I2cAddr.create8bit(0x70)); //Swaping the second color sensor to a new ip, this is the sensor on top.
        //sensorGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("G1");

        one = hardwareMap.dcMotor.get("M1");
        two = hardwareMap.dcMotor.get("M2");
        three = hardwareMap.dcMotor.get("M3");
        four = hardwareMap.dcMotor.get("M4");
        Lift = hardwareMap.dcMotor.get("SL");
        Grabber = hardwareMap.dcMotor.get("SG");
        LeftPusher = hardwareMap.servo.get("SSL");
        RightPusher = hardwareMap.servo.get("SSR");

        LeftLaunch = hardwareMap.dcMotor.get("LL");
        RightLaunch = hardwareMap.dcMotor.get("RL");

        //sensorGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        LeftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightLaunch.setDirection(DcMotor.Direction.REVERSE);
        four.setDirection(DcMotor.Direction.REVERSE);
        three.setDirection(DcMotor.Direction.REVERSE);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMaxDriveTicksPerSec(4000);

        setDrivePower(0,0,0); // Set all motors to zero power

        sensorRGB_One.enableLed(false);
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

    public void setDriveMode(DcMotor.RunMode mode){
        one.setMode(mode);
        two.setMode(mode);
        three.setMode(mode);
        four.setMode(mode);
    }

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior beh){
        one.setZeroPowerBehavior(beh);
        two.setZeroPowerBehavior(beh);
        three.setZeroPowerBehavior(beh);
        four.setZeroPowerBehavior(beh);
    }

    public double setDriveSpeed(double vx, double vy,double va){
        return setDrivePower(vx * TICKS_PER_CM / (Math.sqrt(2.0)*one.getMaxSpeed()),
                             vy * TICKS_PER_CM / (Math.sqrt(2.0)*one.getMaxSpeed()),
                            (va * TURN_RADIUS*COS_BETA*TICKS_PER_CM)/one.getMaxSpeed());
    }

    public void setMaxDriveTicksPerSec(int TicksPerSec){
        one.setMaxSpeed(TicksPerSec);
        two.setMaxSpeed(TicksPerSec);
        three.setMaxSpeed(TicksPerSec);
        four.setMaxSpeed(TicksPerSec);
    }

    public int[] getMaxDriveTicksPerSec (){
        int[] returnVal = new int[4];
        returnVal[0] = one.getMaxSpeed();
        returnVal[1] = two.getMaxSpeed();
        returnVal[2] = three.getMaxSpeed();
        returnVal[3] = four.getMaxSpeed();
        return returnVal;
    }

    public boolean isRightBeaconRed(){
        float red = sensorRGB_One.red();
        float blue = sensorRGB_One.blue();
        float alpha = sensorRGB_One.alpha();
        float green = sensorRGB_One.green();
        return(red >= MIN_RED && !(red == MAX_SENSOR_VALUES && blue == MAX_SENSOR_VALUES && alpha == MAX_SENSOR_VALUES && green == MAX_SENSOR_VALUES));
    }

    public boolean isRightBeaconBlue(){
        float red = sensorRGB_One.red();
        float blue = sensorRGB_One.blue();
        float alpha = sensorRGB_One.alpha();
        float green = sensorRGB_One.green();
        return(blue >= MIN_BLUE && !(red == MAX_SENSOR_VALUES && blue == MAX_SENSOR_VALUES && alpha == MAX_SENSOR_VALUES && green == MAX_SENSOR_VALUES));
    }

    public float[] getColorValues(){
        float[] colorValues = {sensorRGB_One.red(),sensorRGB_One.green(),sensorRGB_One.blue()};
       return colorValues;
    }

    public void updateOdometry(){
        last_Wheel_Ticks.put(0,one.getCurrentPosition());
        last_Wheel_Ticks.put(1,two.getCurrentPosition());
        last_Wheel_Ticks.put(2,three.getCurrentPosition());
        last_Wheel_Ticks.put(3,four.getCurrentPosition());
        //last_Gyro_Theta = (float)((double)-sensorGyro.getIntegratedZValue()*(Math.PI/180.0));
    }

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

}

