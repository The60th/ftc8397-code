package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class OmniBot
{
    //Robot constants
    public final double TICKS_PER_MOTOR_ROTATION =1440;
    public final double GEAR_RATIO = 1.0;  //Motor rotations per wheel rotation.
    public final double WHEEL_DIAMETER = 4.0*2.54;  //4in converted to cm.
    public final double TICKS_PER_CM = TICKS_PER_MOTOR_ROTATION*GEAR_RATIO/(Math.PI*WHEEL_DIAMETER);
    public final double LENGTH = 36.8; //cm
    public final double WIDTH = 30.5;  //cm
    /* Public OpMode members. */
    public DcMotor one;
    public DcMotor two;
    public DcMotor three;
    public DcMotor four;
    public DcMotor Lift;
    public DcMotor Grabber;
    public DcMotor LeftLaunch;
    public DcMotor RightLaunch;

    /* local OpMode members. */
    HardwareMap hardwareMap          =  null;


    /* Constructor */
    public OmniBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
       hardwareMap = ahwMap;

        one = hardwareMap.dcMotor.get("M1");
        two = hardwareMap.dcMotor.get("M2");
        three = hardwareMap.dcMotor.get("M3");
        four = hardwareMap.dcMotor.get("M4");
        Lift = hardwareMap.dcMotor.get("SL");
        Grabber = hardwareMap.dcMotor.get("SG");

        LeftLaunch = hardwareMap.dcMotor.get("LL");
        RightLaunch = hardwareMap.dcMotor.get("RL");

        LeftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        RightLaunch.setDirection(DcMotor.Direction.REVERSE);
        four.setDirection(DcMotor.Direction.REVERSE);
        three.setDirection(DcMotor.Direction.REVERSE);

        setDrivePower(0,0,0);


        // Set all motors to zero power

    }

    public double setDrivePower(double px, double py,double pa){
        double w1 = -px + py -pa;
        double w2 = px + py -pa;
        double w3 = -px + py +pa;
        double w4 = px + py + pa;
        double max = Math.max(Math.abs(w1),Math.abs(w2));
        max = Math.max(max,Math.abs(w3));
        max = Math.max(max,Math.abs(w4));
        max = Math.max(max,1.0);
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


}

