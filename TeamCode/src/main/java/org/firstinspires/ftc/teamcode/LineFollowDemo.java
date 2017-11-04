package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final float LINE_FOLLOW_SPEED = 20; //20 centimeters per second.
    private final float LINE_FOLLOW_ANGLE_FACTOR = 30.0f * ((float)Math.PI/180.0f); //30.0 Degrees converted to radians.

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        followLine(Side.LEFT, bot.sensorMRColor);
        //bot.sensorMRColor is mounted on the left side of the robot.
        //bot.sensorMRColor2 is mounted on the right side of the robot.
    }

    private void followLine(Side side, ColorSensor colorSensor){
        float vXSpeed;
        float vYSpeed;
        boolean onTape = onLine(colorSensor);
        float initAngle = side == Side.LEFT ? INNER_TAPE_ANGLE_RADS : -INNER_TAPE_ANGLE_RADS; //Check if the side is equal to left or right and set it the negative if right.
        float currentAngle = initAngle;
        DemoSignEnum sign;
        ElapsedTime et = new ElapsedTime();

        vXSpeed = -LINE_FOLLOW_SPEED * (float)Math.sin(currentAngle);
        vYSpeed = LINE_FOLLOW_SPEED * (float)Math.cos(currentAngle);
        telemetry.addData("Driving at speeds of for first section: ", "X %f Y %f, Angle 0",vXSpeed,vYSpeed);
        telemetry.addData("Starting onTape?",onTape);
        telemetry.update();
        sleep(500);
        bot.setDriveSpeed(vXSpeed,vYSpeed,0);
        while (opModeIsActive()){
            String calledMethod = "";
            boolean refreshedOnTape = onLine(colorSensor);
            telemetry.addData("OnTape? ",refreshedOnTape);
            if(onTape != refreshedOnTape){
                calledMethod = "onTape!=refreshedTape";
                //On-Off tape state has changed.
                float angleChange = currentAngle - initAngle;
                initAngle = initAngle +  angleChange/2.0f;
                currentAngle = initAngle;
                telemetry.addData("ET value before reset.",et.seconds());
                et.reset();
                onTape = refreshedOnTape;
                if(side == Side.LEFT){
                    sign = DemoSignEnum.Pos;
                }else{
                    sign = DemoSignEnum.Neg;
                }
            }else{
                telemetry.addData("ET value without reset.",et.seconds());
                //On-Off tape state has not changed.
                if(onTape){
                    calledMethod = "if(onTape)";
                    currentAngle = side == Side.LEFT ? currentAngle + (float) et.seconds()*LINE_FOLLOW_ANGLE_FACTOR : currentAngle - (float) et.seconds()*LINE_FOLLOW_ANGLE_FACTOR;
                    telemetry.addData("currentAngle within onTape ",currentAngle);
                    if(side == Side.LEFT){
                        sign = DemoSignEnum.Pos;
                    }else{
                        sign = DemoSignEnum.Neg;
                    }
                    //Check what side of the line you are on then add or remove from the angle to drive off the land.
                }else{
                    calledMethod = "else{}";
                    currentAngle = side == Side.LEFT ? currentAngle - (float) et.seconds()*LINE_FOLLOW_ANGLE_FACTOR : currentAngle + (float) et.seconds()*LINE_FOLLOW_ANGLE_FACTOR;
                    telemetry.addData("currentAngle within onTape -> else ",currentAngle);
                    if(side == Side.LEFT){
                        sign = DemoSignEnum.Neg;
                    }else{
                        sign = DemoSignEnum.Pos;
                    }
                    //Check what side of the line you are on then add or remove from the angle to drive off the land.
                }
            }
            //TODO note to ask jim? It seems like as the current angle value surpasses 180 the value of current angle's sin will become negative, this then causes the robot
            //TODO just to wiggle around rather then driving one way.
            float vXSpeedMod = (float)Math.sin(currentAngle);
            float vYSpeedMod = (float)Math.cos(currentAngle);
            if(sign == DemoSignEnum.Neg){
                vXSpeedMod = Math.abs(vXSpeedMod);
                vYSpeedMod = Math.abs(vYSpeedMod)*-1.0f;
            }else{
                vXSpeedMod = Math.abs(vXSpeedMod);
                vYSpeedMod = Math.abs(vYSpeedMod);
            }
            telemetry.addData("vXSpeedMod " + vXSpeedMod + "vYSpeedMod " + vYSpeedMod,"");
            vXSpeed = LINE_FOLLOW_SPEED * vXSpeedMod;// (float)Math.sin(currentAngle); //Forcing sign to be positive for testing.
            vYSpeed = LINE_FOLLOW_SPEED * vYSpeedMod;//(float)Math.cos(currentAngle);
            telemetry.addData("What method did we call? ",calledMethod);
            telemetry.addData("Driving at speeds of second section: ", "X %f Y %f Angle 0",vXSpeed,vYSpeed,0);
            telemetry.update();
            bot.setDriveSpeed(0,0,0);
            //sleep(500); //Add a 500 mili second wait to debug better.
            bot.setDriveSpeed(vXSpeed*2,vYSpeed,0);
        }
    }

    private boolean onLine(ColorSensor colorSensor){
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8,colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues[1] > .5;
    }
}
