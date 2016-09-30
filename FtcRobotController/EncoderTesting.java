package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by CanAdirondack on 4/26/2016.
 */
public class EncoderTesting extends OpMode
{
    DcMotor Left_Motor;
    DcMotor Right_Motor;
    DcMotor Arm1;
    DcMotor Arm2;
    Servo LeftSweep;
    Servo RightSweep;

    public void init(){
    int pos;
        hardwareMap.logDevices();

        Left_Motor = hardwareMap.dcMotor.get("LM");
        Right_Motor = hardwareMap.dcMotor.get("RM");

        Left_Motor.setDirection(DcMotor.Direction.REVERSE);
        Right_Motor.setDirection(DcMotor.Direction.REVERSE);

       // Left_Motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
       // Right_Motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    }
    public void loop(){
        double power ;
                power = gamepad1.left_stick_y;
        //Left_Motor.setTargetPosition(1220);//1220 seems to be the number of rotations it takes to get one axle turn.
       // Left_Motor.setPower(1);
        //Right_Motor.setTargetPosition(1220);
        //Right_Motor.setPower(1)
        if(gamepad1.left_stick_y >= .01){
            Left_Motor.setPower(1);
        }
        int pos = Left_Motor.getCurrentPosition();
       telemetry.addData("Position: " ,pos/(3*420));
        //Left_Motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //Right_Motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }
}