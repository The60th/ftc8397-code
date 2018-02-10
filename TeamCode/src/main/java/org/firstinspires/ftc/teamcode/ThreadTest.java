package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by FTC Team 8397 on 1/28/2018.
 */
@TeleOp(name = "Test", group = "asd")
@Disabled

public class ThreadTest extends LinearOpMode {
    boolean newThread = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Ready to start. " , "...");
        telemetry.update();

        waitForStart();

        if(!newThread){
            telemetry.addData("Starting thread.","");
            telemetry.update();
            newThread = true;
            final int[] test = {1};
            new Thread(new Runnable() {
                public void run() {
                    while (opModeIsActive()) {
                        test[0] = test[0] + 1;
                        telemetry.addData("Gamepad1 Thread A: ", gamepad1.a);
                        telemetry.addData("Runable data: ", test[0]);
                    }
                }
            }).start();
        }
        int x = 1;
        while (opModeIsActive()){
            telemetry.addData("Gamepad1 A: ", gamepad1.a);
            telemetry.addData("Non thread: ", x);
            x=x+1;
            telemetry.update();
        }
    }
}
