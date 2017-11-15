package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by FTC Team 8397 on 10/27/2017.
 */
@Autonomous(name = "TestLineFollowDemo", group = "Test")
public class LineFollowDemo extends LinearOpMode {
    private MechBotSensor bot = new MechBotSensor();
    private enum Side {LEFT,RIGHT}
    private enum TapeColor {RED,BLUE}
    enum DemoSignEnum {Neg,Pos}
    private final float INNER_TAPE_ANGLE = 33.70f;
    private final float INNER_TAPE_ANGLE_RADS = INNER_TAPE_ANGLE * ((float)Math.PI/180.0f);
    private final float LINE_FOLLOW_SPEED = 10.0f; //10 centimeters per second.
    private final float LINE_FOLLOW_ANGLE_FACTOR = 30.0f * ((float)Math.PI/180.0f); //30.0 Degrees converted to radians.
    private final float HEADING_CORECTION_FACTOR = 2.0f;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        followLineProportionate(Side.LEFT, bot.sensorMRColor);
        //followLine(Side.LEFT, bot.sensorMRColor);
        //turnAngleGyro(90f,3,.3f);
        //bot.sensorMRColor is mounted on the left side of the robot.
        //bot.sensorMRColor2 is mounted on the right side of the robot.
    }



    private void followLine(Side side, ColorSensor colorSensor){
        final String TAG = "FOLLOW_LINE";
        float vXSpeed;
        float vYSpeed;
        float vASpeed;
        boolean onTape = onLine(colorSensor);
        float initAngle = side == Side.LEFT ? INNER_TAPE_ANGLE_RADS : -INNER_TAPE_ANGLE_RADS; //Check if the side is equal to left or right and set it the negative if right.
        float currentAngle = initAngle;
        float heading = bot.getHeadingRadians();

        RobotLog.dd(TAG, "initAngle = %.1f, initHeading = %.1f", initAngle * 180.0/Math.PI, heading * 180.0/Math.PI);

        ElapsedTime et = new ElapsedTime();

        vXSpeed = -LINE_FOLLOW_SPEED * (float)Math.sin(currentAngle);
        vYSpeed = LINE_FOLLOW_SPEED * (float)Math.cos(currentAngle);
        vASpeed = -heading*HEADING_CORECTION_FACTOR;

        telemetry.addData("Driving at speeds of for first section: ", "X %f Y %f, Angle 0",vXSpeed,vYSpeed);
        telemetry.addData("Starting onTape?",onTape);
        telemetry.update();

        bot.setDriveSpeed(vXSpeed,vYSpeed,vASpeed);
        while (opModeIsActive()){

            //String calledMethod = "";
            boolean refreshedOnTape = onLine(colorSensor);
            RobotLog.dd(TAG+"-TAPE_STATUS", "OnTape? " + refreshedOnTape);
            telemetry.addData("OnTape? ",refreshedOnTape);
            if(onTape != refreshedOnTape){
                //calledMethod = "onTape!=refreshedTape";
                //On-Off tape state has changed.
                float angleChange = currentAngle - initAngle;
                initAngle = initAngle +  angleChange/2.0f;
                currentAngle = initAngle;
                telemetry.addData("ET value before reset.",et.seconds());
                et.reset();
                onTape = refreshedOnTape;
               /* if(side == Side.LEFT){
                    sign = DemoSignEnum.Pos;
                }else{
                    sign = DemoSignEnum.Neg;
                }*/
            }else{
                telemetry.addData("ET value without reset.",et.seconds());
                //On-Off tape state has not changed.
                if(onTape){
                   // calledMethod = "if(onTape)";
                    currentAngle = side == Side.LEFT ? initAngle + (float) et.seconds()*LINE_FOLLOW_ANGLE_FACTOR : initAngle - (float) et.seconds()*LINE_FOLLOW_ANGLE_FACTOR;
                    telemetry.addData("currentAngle within onTape ",currentAngle);
                    /*if(side == Side.LEFT){
                        sign = DemoSignEnum.Pos;
                    }else{
                        sign = DemoSignEnum.Neg;
                    }*/
                    //Check what side of the line you are on then add or remove from the angle to drive off the land.
                }else{
                    //calledMethod = "else{}";
                    currentAngle = side == Side.LEFT ? initAngle - (float) et.seconds()*LINE_FOLLOW_ANGLE_FACTOR : initAngle + (float) et.seconds()*LINE_FOLLOW_ANGLE_FACTOR;
                    telemetry.addData("currentAngle within onTape -> else ",currentAngle);
                   /* if(side == Side.LEFT){
                        sign = DemoSignEnum.Neg;
                    }else{
                        sign = DemoSignEnum.Pos;
                    }*/
                    //Check what side of the line you are on then add or remove from the angle to drive off the land.
                }
            }
            //TODO note to ask jim? It seems like as the current angle value surpasses 180 the value of current angle's sin will become negative, this then causes the robot
            //TODO just to wiggle around rather then driving one way.
            /*if(sign == DemoSignEnum.Neg){
                vXSpeedMod = Math.abs(vXSpeedMod);
                vYSpeedMod = Math.abs(vYSpeedMod)*-1.0f;
            }else{
                vXSpeedMod = Math.abs(vXSpeedMod);
                vYSpeedMod = Math.abs(vYSpeedMod);
            }*/
            heading = bot.getHeadingRadians();
            vXSpeed = LINE_FOLLOW_SPEED * -(float)Math.sin(currentAngle);// (float)Math.sin(currentAngle); //Forcing sign to be positive for testing.
            vYSpeed = LINE_FOLLOW_SPEED * (float)Math.cos(currentAngle);//(float)Math.cos(currentAngle);
            vASpeed = -heading * HEADING_CORECTION_FACTOR;



            RobotLog.dd(TAG, "currAngle = %.1f, Heading = %.1f", currentAngle * 180.0/Math.PI, heading * 180.0/Math.PI);

            telemetry.addData("Driving at speeds of second section: ", "X %f Y %f Angle 0",vXSpeed,vYSpeed,0);
            telemetry.update();
            bot.setDriveSpeed(vXSpeed,vYSpeed,vASpeed);
        }
    }

    private boolean onLine(ColorSensor colorSensor){
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8,colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues[1] > .5;
    }

    private void followLineProportionate(Side side, ColorSensor colorSensor){
        float[] hsvValues = new float[3];
        final float coeff = 20.0f;
        while (opModeIsActive()){
            float heading = bot.getHeadingRadians();
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            float err = side == Side.LEFT? 0.5f - hsvValues[1] : hsvValues[1] - 0.5f;
            if (err < -0.5) err = -0.4f;
            else if (err > 0.5) err = 0.4f;
            float angleDiff = side == Side.LEFT? heading - INNER_TAPE_ANGLE_RADS : heading + INNER_TAPE_ANGLE_RADS;
            float vx = LINE_FOLLOW_SPEED * (float)Math.sin(angleDiff) + coeff * err * (float)Math.cos(angleDiff);
            float vy = LINE_FOLLOW_SPEED * (float)Math.cos(angleDiff) - coeff * err * (float)Math.sin(angleDiff);
            float va = -heading * HEADING_CORECTION_FACTOR;
            bot.setDriveSpeed(vx, vy, va);
        }
    }

    //Using gyro, turns robot the specified number of degrees (angle), with acceptable error
    //of +/- tolerance degrees, assuming a latency between new gyro readings of "latency" seconds
    protected void turnAngleGyro(float angle, float tolerance, float latency) {
        //Tolerance in degrees latency seconds.
        //Convert to radians.
        angle = angle * (float)Math.PI/180f;
        tolerance = tolerance * (float)Math.PI/180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 0.2f * (float)Math.PI;
        float heading = bot.getHeadingRadians();
        float targetHeading = heading + angle;
        float offset = (float)VuMarkNavigator.NormalizeAngle(targetHeading - heading);

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            RobotLog.a("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
            heading = bot.getHeadingRadians();
            offset = targetHeading - heading;
        }
        bot.setDrivePower(0, 0, 0);
    }

    //Turns robot to a specific integratedZ heading using Gyro, targetHeading in degrees
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency){
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float)Math.PI/180f;
        targetHeading = targetHeading * (float)Math.PI/180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 0.75f * (float)Math.PI;
        float heading = bot.getHeadingRadians();
        float offset = (float)VuMarkNavigator.NormalizeAngle(targetHeading - heading);

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            RobotLog.a("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
            heading = bot.getHeadingRadians();
            offset = targetHeading - heading;
        }
        bot.setDrivePower(0, 0, 0);

    }

    protected void driveStraightGyroTime(float vx, float vy, float duration) {
        final float C_ANGLE = 2.0f;
        final float initialHeading = bot.getHeadingRadians();
        ElapsedTime et = new ElapsedTime();
        double scale = bot.setDriveSpeed(vx, vy, 0);
        RobotLog.a("<Debug> DriveStrGyroTime Pre Scale duration = %.0f vx = %.0f vy = %.0f", duration, vx, vy);

        duration /= scale;
        vx *= scale;
        vy *= scale;
        RobotLog.a("<Debug> DriveStrGyroTime Post Scale( %.2f ) duration = %.0f vx = %.0f vy = %.0f", scale, duration, vx, vy);

        while(opModeIsActive()){
            double etms = et.milliseconds();
            if(etms > duration) break;
            float currentHeading = bot.getHeadingRadians();
            float va = (initialHeading - currentHeading) * C_ANGLE;
            RobotLog.a("<Debug> Initial Angle = %.1f Current Angle = %.1f", initialHeading, currentHeading);
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0, 0);

    }
}
