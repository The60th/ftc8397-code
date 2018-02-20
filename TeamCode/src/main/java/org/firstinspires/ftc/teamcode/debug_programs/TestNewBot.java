package org.firstinspires.ftc.teamcode.debug_programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;


/**
 * Created by FTC Team 8397 on 2/20/2018.
 */
@TeleOp(name = "TestNewBot", group = "Tele opmode")
public class TestNewBot extends LoggingLinearOpMode {

    DcMotor  rightIntake;
    DcMotor  leftIntake;

    public void runLoggingOpmode() throws InterruptedException {
        rightIntake = hardwareMap.dcMotor.get("RI");
        leftIntake = hardwareMap.dcMotor.get("LI");

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){


            if (gamepad1.right_stick_y > .05){
                rightIntake.setPower(-.5);
            }
            else if (gamepad1.right_stick_y < -.05){
                rightIntake.setPower(.5);
            }
            else{rightIntake.setPower(0);}
            if (gamepad1.left_stick_y > .05){
                leftIntake.setPower(-.5);
            }
            else if (gamepad1.left_stick_y < -.05){
                leftIntake.setPower(.5);
            }
            else{leftIntake.setPower(0);}
        }
    }
}
