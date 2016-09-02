package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftccommon.DbgLog;


//Created by TeamBeta8397 on 11/25/2015.

//This program has been made by members of the FTC Team, Beta8397 for the 2016 competition year and the challenge First-Resq this is only one of many
//programs team Beta's software development team worked on through the year, this one in practicality is the program we will use for competition teleop.


public class TeleOp extends OpMode
{
    //Here we first have a set of variable deculations for the different DC and Servo motors we will be using threw out the program.

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor upMiddleMotor;
    DcMotor threeArmMotor;
    DcMotor oneArmMotor;
    DcMotor twoArmMotor;
    Servo turnServo;
    Servo dumpServo;
    @Override
    public void init()
    {
        //Here we have commands that are changing the declared names for are motors from above to what the need to be checked for in the configurations files
        //this is so the names are easyer to work with threw out the whole program and in the configuration files.
        leftMotor = hardwareMap.dcMotor.get("LM1"); //controller one.
        rightMotor = hardwareMap.dcMotor.get("RM1"); //controller one.

        upMiddleMotor = hardwareMap.dcMotor.get("MM1");//controller two Drive wheels.
        oneArmMotor = hardwareMap.dcMotor.get("AM1");//controller two Drive wheels.

        threeArmMotor = hardwareMap.dcMotor.get("AM3");//Controller three.
        twoArmMotor = hardwareMap.dcMotor.get("AM2");//controller three.

        turnServo = hardwareMap.servo.get("TS1");//Servo controller one.
        dumpServo = hardwareMap.servo.get("DS1"); //Servo controller one.


        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
//This is the start of our void loop where we run all of are driver controls commands.
//Inside of here we have all the commands for the drive wheels, servo(Dump and Turn table), and all the needed arm controls to
//to let are robot run correctly.
    public void loop()
    {
       //Right below here is all the variables that we use in a program, mostly they are all doubles which are a 64-bit IEEE 754 floating point.
       //As you can see many the variables are set of the values of a joystick then edited, this is so we can create a floating point value that changes
       //as the joystick moves around to. With this one could set the power of a motor to a variables then have that be updated in live as the joystick moves.
        double LeftDrive = gamepad1.left_stick_y/2.5;
        double RightDrive = -gamepad1.right_stick_y/2.5;

        double JoyOneLeft = gamepad1.left_stick_y/1.25;
        double JoyOneRight = -gamepad1.right_stick_y/1.25;

        double rightX = -gamepad2.left_stick_y/2;
        double leftX = -gamepad2.right_stick_y/2;

        double JoyTwoLeft = -gamepad2.left_stick_y/1.25;
        double JoyTwoRight = -gamepad2.right_stick_y/1.25;

       //Here you can first see where we have the motors be set to the power of a variables that is taken from the joystick. In this it is the two variables leftX
       //and rightX which are both set to the power of gamepad2 lefts joystick and gamepad2 rights joystick and then divide by the integer 2 to get a lower value.
        leftMotor.setPower(leftX);
        rightMotor.setPower(rightX);

        //Turbo mode.
        //Here now is the start of some are more advanced code in drive control. To shortly explain what this does is that when both joysticks are fully pushed
        //forward or pulled back and are opposites(As in one is forward and one is back so on) it increases the power of the motors from the joystick value
        //divided by 2.5 to a value only divided by 1.25. So if the joysticks are in this position its already turning the robot so the purpose of this is
        //to basically double the turn speed so we have more control over the robot turning teleop.
        if(LeftDrive >=.35 && RightDrive >= -.35  && RightDrive != 0 && !gamepad1.right_stick_button && !gamepad1.left_stick_button
                || LeftDrive <= -.35 && RightDrive <= -.35 && RightDrive != 0 && !gamepad1.right_stick_button && !gamepad1.left_stick_button)
        {
            LeftDrive = 0;
            RightDrive = 0;
            oneArmMotor.setPower(JoyOneRight);
            upMiddleMotor.setPower(JoyOneLeft);
        }
        //This is more of the above code for what is pretty much are turbo mode, this contuines on from before. By checking if the joystick buttons on controller
        //one are both pushed down, if they are it switch the value you being divided by 2.5 to 1.25 working pretty much the same as above but not only for turns.
        //This is to give us more speed and power for when we need to move the robot fast or to climb the mountain.
        else if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            oneArmMotor.setPower(JoyOneRight);
            upMiddleMotor.setPower(JoyOneLeft);
        }
        else{
            oneArmMotor.setPower(RightDrive);
            upMiddleMotor.setPower(LeftDrive);
        }

       //This is the same as the last section of code other then the fact it drives the arm of the robot up and down and both buttons do not need to be pushed
       //at the same time for this to work as they do above. This is again to give us more speed and control of the robot when needed.
        if(gamepad2.left_stick_button && !gamepad2.right_stick_button)
        {
           leftMotor.setPower(JoyTwoLeft);
        }

        else if(gamepad2.right_stick_button && !gamepad2.left_stick_button)
        {
            rightMotor.setPower(JoyTwoRight);
        }
        else
        {
            leftMotor.setPower(leftX);
            rightMotor.setPower(rightX);
        }

        //Well now that we have gone through all of the turbo and non turbo controls for the arm and drive wheels, now we move on to the other moving parts of
        //the robot.


        //This here is the commands for the motor that runs the gear box that can lift changes the arm's angle up and down. It runs by pressing gamepad2's
        //right bumper to lift it up then the left bumper to bring it down. As you can see the values on here are set to a fixed amount rather then a floating
        //point value like a double, this is because the bumper is simply a bottom and can only handle boolean logic(Ture or False, Pressed or not Pressed)
        //and as such there is no good way to set a power depending on how much the button is pressed.
      if (gamepad2.right_bumper)

        {
            threeArmMotor.setPower(.5);
        }
        else if(gamepad2.left_bumper)
        {
            threeArmMotor.setPower(-.5);
        }
        else
        {
            threeArmMotor.setPower(0.0);
        }

        //Now we have the commands for the lift in the front of the robot this is one of the most important parts of the robot as its the only true way to
        //score a large amount of points that we need to place well. Sense the software is pretty simple we can first explain a bit of the hardware behind it.
        //The hardware behind it runs off of one motor that is connected to a chain that spins the sprockets from a textrix track kit, that spins the tracks which
        //have plastic inserts to trap and pick up balls and blocks from the ground and bring them up to are bucket to be collected.
        //The software behind this is pretty much the same as seen above it checks to see if a bumper is pressed and if it is it spins the motor and chain in a
        //way to bring the blocks and other scoring elements up, you press the other bumper and it brings the scoring elements down. The down part is mostly
        //so the scoring elements have a less chance of getting stuck on parts of the robot and we have a way to free them.

        if (gamepad1.left_bumper)
        {
            twoArmMotor.setPower(.45);
        }
        else if(gamepad1.right_bumper)
        {
            twoArmMotor.setPower(-.45);
        }
        else
        {
            twoArmMotor.setPower(0.0);
        }

        //Now we are at last moving away from DC motors, all the code before this has been to run DC motors to dive the robot, the arm and so on. Now we are going
        //to be working with servos, servos have one major difference from DC motor that is the servo knows what its position is while a DC does not, because of
        //this you run a servo to a set position not a power.

        //Here we have our turntable servo that is chained and geared up to be given more power to be able to spin the table. It runs off of the second gamepad's
        //left and right triggers and because triggers give a floating point value not a boolean we have to check to see if its greater or less then something
        //not just true as we do with bumpers. That the reason it checks if greater then .80(The range is 0-1) or equal not just true or false.

        if(gamepad2.left_trigger >= .80)
        {
            turnServo.setPosition(0);
        }
         else if(gamepad2.right_trigger >= .80)
        {
            turnServo.setPosition(1);
        }
        else
        {
          turnServo.setPosition(.5);
        }

        //Now we have are last command before debug info. This is going back to the boolean button logic talked about a bit before where we just check to see
        //if a button is press or not(True or False) rather then it meeting or beating a set threshold for its value. This just checks to see if gamepad2's
        //X button is pressed or not, if it is it moves the servo to a position that flips are bucket over to dump any scoring elements inside of it to score.
        //If its not pressed it just leaves it at is rest position, the needed positions for a servo really depend on what type you are using and how they are
        //mounted on your bot.
        if(gamepad2.x)
        {
         dumpServo.setPosition(0);
        }
        else
        {
            dumpServo.setPosition(1);
        }


       //Now are are pretty much at the end of our program with the last few lines now being some telemetry data for debugging. The way we do most of our debugging
       //is by viewing values live as they update so we have the best idea on whats happening. Another way to do it is to use the logcat text files which is
       //something some of are team members are currently working with. So the way this works is we just have a string for the display name which is that
       //listed with a value following it, the value is taken from a object in the code usually a motor or servo, sometimes something else and is then displayed
       //to the FTC Driver Station app.

        telemetry.addData("Turn Servo's current position is:",turnServo.getPosition());
        telemetry.addData("Dump Servo's current position is:",dumpServo.getPosition());
        telemetry.addData("left motor power:",leftMotor.getPower());
        telemetry.addData("MM1 power:",upMiddleMotor.getPower());
        telemetry.addData("right motor power:",rightMotor.getPower());
        telemetry.addData("1AM power:", oneArmMotor.getPower());
        telemetry.addData("2AM power:", twoArmMotor.getPower());
        telemetry.addData("3AM:", threeArmMotor.getPower());


       //After reading this all if you have any questions feel free to contact any member of FTC Team Beta8397 for more information on how any of this code
       //all are members would be more then happy to walk you threw on how it works!
    }
}







