package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 8/27/2017.
 */
@TeleOp(name="Rev Robotics Demos", group="Rev")
public class RevRoboticsDemo extends OpMode{
    DcMotor motor;
    Servo servo;
    ColorSensor colorSensor;
    BNO055IMU gyro;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        servo = hardwareMap.servo.get("servo");
        colorSensor = hardwareMap.colorSensor.get("sensor");
        gyro = (BNO055IMU) hardwareMap.gyroSensor.get("gyro");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper){
            motor.setPower(1);
            telemetry.addData("Motor driving forward. ","\n Current encoder ticks are: %f",motor.getCurrentPosition());
        }
        else if(gamepad1.right_bumper){
            motor.setPower(-1);
            telemetry.addData("Motor driving backwards. ","\n Current encoder ticks are: %f",motor.getCurrentPosition());
        }
        else{
            motor.setPower(0);
            telemetry.addData("Motor stopped. ","\n Current encoder ticks are: %f",motor.getCurrentPosition());
        }


        if(gamepad1.right_bumper){
        servo.setPosition(1);}
        else if(gamepad1.right_bumper){
        servo.setPosition(-1);}
        else{
        servo.setPosition(0);}

        if(gamepad1.a){
            telemetry.addData("Red: %f Green: %f Blue %f", " ",colorSensor.red(),colorSensor.green(),colorSensor.blue());
        }
        //log
        //forward backwards
        //rgb color sensor on button
        //encoder ticks
        telemetry.addData("Gyro Temp Reading: %f","",gyro.getTemperature());
        telemetry.update();
}
}
