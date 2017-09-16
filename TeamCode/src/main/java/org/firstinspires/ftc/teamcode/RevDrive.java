package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by FTC Team 8397 on 9/16/2017.
 */
@TeleOp(name="DemRevDrive", group="Rev")
public class RevDrive extends LinearOpMode {
    private DcMotor one,two,three,four;
    @Override
    public void runOpMode() throws InterruptedException {
        one = hardwareMap.dcMotor.get("one");
        two = hardwareMap.dcMotor.get("two");
        three = hardwareMap.dcMotor.get("three");
        four = hardwareMap.dcMotor.get("four");
        three.setDirection(DcMotorSimple.Direction.REVERSE);
        four.setDirection(DcMotorSimple.Direction.REVERSE);
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        four.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        four.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.left_stick_y > .2){
                drive(.20);
            }
            else if(gamepad1.left_stick_y < -.2){
                drive(-.20);
            }
            else if(gamepad1.left_stick_x > .2){
                one.setPower(.20);
                two.setPower(-.20);
                three.setPower(.20);
                four.setPower(-.20);
            }
            else if(gamepad1.left_stick_x < -.2){
                one.setPower(-.20);
                two.setPower(.20);
                three.setPower(-.20);
                four.setPower(.20);
            }
            else {
                drive(0);
            }
            telemetry.addData("Motor encoders: ","One: %d Four: %d",one.getCurrentPosition(),four.getCurrentPosition());
            telemetry.update();
        }


    }
    public void drive(double power){
        one.setPower(power);
        two.setPower(power);
        three.setPower(power);
        four.setPower(power);
    }
}
