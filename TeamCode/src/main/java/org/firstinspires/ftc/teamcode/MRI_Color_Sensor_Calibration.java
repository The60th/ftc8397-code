package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.hardware.I2cAddr;
        import com.qualcomm.robotcore.hardware.I2cDevice;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
        import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Color Calibration", group="MRI")
public class MRI_Color_Sensor_Calibration extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorCcache;

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        colorC = hardwareMap.i2cDevice.get("cc");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x70), false);
        //        sensorRGB.setI2cAddress(I2cAddr.create8bit(0x70));

        colorCreader.engage();

        colorCreader.write8(3, 0);    //Set the mode of the color sensor to Active


        telemetry.addData("HELLO", "Point the color sensor into open space. Then press A");
        telemetry.addData("HELLO", "Custom Calibration");

        telemetry.update();
        while (!gamepad1.a) {

        }

        colorCreader.write8(3, 66); //Black Level Calibration
        telemetry.addData("HELLO", "Wait while Black Level Calibration");
        telemetry.update();
        colorCcache = colorCreader.read(0x04, 1);
        while (colorCcache[0] == 66) {
            colorCcache = colorCreader.read(0x04, 1);
        }

        while (gamepad1.a) {
            telemetry.addData("HELLO", "Release the A button");
            telemetry.update();
        }


        telemetry.addData("HELLO", "Point the color sensor 5cm or 2in away at a white target. Then press A");
        telemetry.update();
        while (!gamepad1.a) {

        }
        colorCreader.write8(3, 67); //Black Level Calibration
        telemetry.addData("HELLO", "Wait while White Balance Calibration");
        telemetry.update();
        colorCcache = colorCreader.read(0x04, 1);
        while (colorCcache[0] == 67) {
            colorCcache = colorCreader.read(0x04, 1);
        }

        telemetry.addData("White Calibration Complete", "");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.x) {
                colorCreader.write8(3, 0);    //Set the mode of the color sensor to Active
                while (gamepad1.x) {
                }
            } else if (gamepad1.y) {
                colorCreader.write8(3, 1);    //Set the mode of the color sensor to Passive
                while (gamepad1.y) {
                }
            }

            colorCcache = colorCreader.read(0x04, 1);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("1 C#", colorCcache[0] & 0xFF);
            telemetry.addData("2", "Press X for Active");
            telemetry.addData("3", "Press Y for Passive");
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}