package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TestFunctions;
import org.firstinspires.ftc.teamcode.TestFunctions;

/**
 * Created by CanAdirondack on 4/26/2016.
 */
public class EAFD_Robot extends OpMode
{
    DcMotor Left_Motor;
    DcMotor Right_Motor;
    DcMotor Arm1;
    DcMotor Arm2;
    Servo LeftSweep;
    Servo RightSweep;

    public void init(){
        TestFunctions.Hello();

        hardwareMap.logDevices();

        Left_Motor = hardwareMap.dcMotor.get("LM");
        Right_Motor = hardwareMap.dcMotor.get("RM");
        Arm1 = hardwareMap.dcMotor.get("A1");
        Arm2 = hardwareMap.dcMotor.get("A2");
        LeftSweep = hardwareMap.servo.get("LS");
        RightSweep = hardwareMap.servo.get("RW");

        Right_Motor.setDirection(DcMotor.Direction.REVERSE);
        Arm2.setDirection(DcMotor.Direction.REVERSE);


    }
    public void loop(){
        double LeftDrive;
        double RightDrive;
        double ArmPower;
        //Catapult mode!
       // if(gamepad1.a && gamepad1.b && gamepad2.a && gamepad2.b){
        //    ArmPower = gamepad2.left_stick_y;
        //    LeftDrive = gamepad1.left_stick_y;
        //    RightDrive = gamepad1.right_stick_y;
       // }
        //Default mode with edited power values
       // else{

            LeftDrive = gamepad1.left_stick_y;
            RightDrive = gamepad1.right_stick_y;
            ArmPower = gamepad2.left_stick_y/4;
       // }
        //Drive controls
        Left_Motor.setPower(LeftDrive);
        Right_Motor.setPower(RightDrive);

        //For arm hcontrols

        if(gamepad2.left_stick_y > (.20) && gamepad2.x) {
            Arm1.setPower(ArmPower);
            Arm2.setPower(ArmPower);
            telemetry.addData("Running arm forwards", "");
        }
        else if(gamepad2.left_stick_y< - (.20) && gamepad2.x){
            Arm1.setPower(ArmPower);
            Arm2.setPower(ArmPower);
            telemetry.addData("Running arm backwards","");
        }
        else{
            Arm1.setPower(0);
            Arm2.setPower(0);
        }
        // RightSweep moving code
        if(gamepad1.b){
            RightSweep.setPosition(1);
        }

        else if(gamepad1.x){
            RightSweep.setPosition(0);
        }
        else{
            RightSweep.setPosition(.5);
        }

        if (gamepad1.a) {
            LeftSweep.setPosition(0);
        }
        else if(gamepad1.y){
            LeftSweep.setPosition(1);
        }
        else{
            LeftSweep.setPosition(.5);
        }
    }
}
