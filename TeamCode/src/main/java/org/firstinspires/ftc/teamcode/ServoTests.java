package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;

/**
 * Created by FTC Team 8397 on 12/5/2017.
 */
@TeleOp(name="Servo tests",group = "Testing")
@Disabled
public class ServoTests extends LoggingLinearOpMode {
    private Servo leftLowerClamp, leftUpperClamp, rightLowerClamp, rightUpperClamp;
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        leftLowerClamp = hardwareMap.servo.get("leftLowerClamp");
        leftUpperClamp = hardwareMap.servo.get("leftUpperClamp");

        rightLowerClamp = hardwareMap.servo.get("rightLowerClamp");
        rightUpperClamp = hardwareMap.servo.get("rightUpperClamp");

        rightLowerClamp.setDirection(Servo.Direction.REVERSE);
        rightUpperClamp.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                closeLowerClamp();
            }
            else if(gamepad1.y){
                openLowerClamp();
            }
            else if(gamepad1.x || gamepad1.b){
                midPosLowerClamp();
            }

            if(gamepad2.a){
                closeUpperClamp();
            }
            else if(gamepad2.y){
                openUpperClamp();
            }
            else if(gamepad2.x || gamepad2.b){
                midPosUpperClamp();
            }
        }
    }




    public void closeLowerClamp(){
        leftLowerClamp.setPosition(1);
        rightLowerClamp.setPosition(1);
    }

    public void openLowerClamp(){
        leftLowerClamp.setPosition(0);
        rightLowerClamp.setPosition(0);
    }
    public void midPosLowerClamp(){
        leftLowerClamp.setPosition(.5);
        rightLowerClamp.setPosition(.5);
    }

    public void closeUpperClamp(){
        leftUpperClamp.setPosition(1);
        rightUpperClamp.setPosition(1);
    }

    public void openUpperClamp(){
        leftUpperClamp.setPosition(0);
        rightUpperClamp.setPosition(0);
    }

    public void midPosUpperClamp(){
        leftUpperClamp.setPosition(0.5);
        rightUpperClamp.setPosition(0.5);
    }

}
