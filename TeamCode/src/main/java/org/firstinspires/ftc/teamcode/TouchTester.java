package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;

/**
 * Created by FTC Team 8397 on 4/13/2018.
 */
@TeleOp(name = "touch tester", group = "tester")
@Disabled
public class TouchTester extends LoggingLinearOpMode {
    DigitalChannel digitalTouch;  // Hardware Device Object
    @Override
    public void runLoggingOpmode() throws InterruptedException {

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            BetaLog.dd("Debug",""+digitalTouch.getState());
            telemetry.addData("test","test");
            telemetry.update();
        }
    }
}
