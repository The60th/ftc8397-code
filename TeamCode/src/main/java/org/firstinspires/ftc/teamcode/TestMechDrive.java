package org.firstinspires.ftc.teamcode;

        import android.graphics.Color;

        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
        import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.I2cAddr;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.external.Func;
        import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
        import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

        import java.util.Locale;


/**
 * Created by FTC Team 8397 on 9/22/2017.
 */
@TeleOp(name="TestMechDrive", group="Rev")
public class TestMechDrive extends LinearOpMode {
    private MechBotSensor mechBot = new MechBotSensor();
    private boolean drive = true;
    private boolean sensor = false;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    float hsvValuesMR[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float valuesMR[] = hsvValuesMR;

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;
    //TODO The Guide button does NOT work.
    //TODO The guide button will close the app.
    public void runOpMode() throws InterruptedException {
        mechBot.init(hardwareMap);

        /*mechBot.sensorGyro.calibrate();
        while (mechBot.sensorGyro.isCalibrating()) {
            telemetry.addData("Calibrating Modern Robotics Gyro","");
            telemetry.update();
            idle();
        }*/

        telemetry.addData("Ready to go: ","");
        telemetry.update();
        waitForStart();
        telemetry.addData("Starting","");
        telemetry.update();
        mechBot.sensorMRColor.enableLed(bLedOn);
        while (opModeIsActive()) {
            if(drive) {
                if(gamepad1.a){
                    drive = !drive;
                    sensor = !sensor;

                }
                double x = 0;
                double y = 0;
                double a = 0;
                x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
                y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
                if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
                    if (gamepad1.left_trigger > gamepad1.right_trigger)
                        a = -gamepad1.left_trigger;
                    else {
                        a = gamepad1.right_trigger;
                    }
                }
                telemetry.addData("Drive Mode:","");
                telemetry.addData("","\n");
                telemetry.addData("Driving with speeds. " + x + " x " + y + " y " + a + " a ", "");
                telemetry.update();
                mechBot.setDrivePower(x, y, a);
            }
            else if(sensor){
                if(gamepad2.a){
                    drive = !drive;
                    sensor = !sensor;
                }
                bCurrState = gamepad1.x;

                // check for button state transitions.
                if (bCurrState && (bCurrState != bPrevState))  {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    mechBot.sensorMRColor.enableLed(bLedOn);
                }

                // update previous state variable.
                bPrevState = bCurrState;
                Color.RGBToHSV((int) (mechBot.sensorREVColor.red() * SCALE_FACTOR),
                        (int) (mechBot.sensorREVColor.green() * SCALE_FACTOR),
                        (int) (mechBot.sensorREVColor.blue() * SCALE_FACTOR),
                        hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("***<Sensor Mode>***","");
                telemetry.addData("","\n");

                telemetry.addData("*****Rev Robotics Color Distance*****","");

                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", mechBot.sensorRevDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", mechBot.sensorREVColor.alpha());
                telemetry.addData("Red  ", mechBot.sensorREVColor.red());
                telemetry.addData("Green", mechBot.sensorREVColor.green());
                telemetry.addData("Blue ", mechBot.sensorREVColor.blue());
                telemetry.addData("Hue", hsvValues[0]);
                //Sensor stuff.
                telemetry.addData("","\n");

                telemetry.addData("*****Modern Robotics Color One*****","");
                Color.RGBToHSV(mechBot.sensorMRColor.red() * 8, mechBot.sensorMRColor.green() * 8, mechBot.sensorMRColor.blue() * 8, hsvValuesMR);
                // send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", mechBot.sensorMRColor.alpha());
                telemetry.addData("Red  ", mechBot.sensorMRColor.red());
                telemetry.addData("Green", mechBot.sensorMRColor.green());
                telemetry.addData("Blue ", mechBot.sensorMRColor.blue());
                telemetry.addData("Hue", hsvValuesMR[0]);
                telemetry.addData("","\n");

                telemetry.addData("*****Modern Robotics Color Two*****","");
                telemetry.addData("","\n");

                telemetry.addData("*****Modern Robotics Gyro*****","");
                //telemetry.addData("Heading: ",mechBot.sensorGyro.getHeading());
                telemetry.addData("","\n");

                telemetry.addData("*****Rev Robotics BN055*****","");

                //mechBot.angles   = mechBot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //mechBot.gravity  = mechBot.imu.getGravity();
                telemetry
                        .addData("status",  mechBot.imu.getSystemStatus().toShortString());

                telemetry.addData("calib",  mechBot.imu.getCalibrationStatus().toString());


               // telemetry.addData("heading",  mechBot.formatAngle(mechBot.angles.angleUnit, mechBot.angles.firstAngle));

               // telemetry.addData("roll",  mechBot.formatAngle(mechBot.angles.angleUnit, mechBot.angles.secondAngle));

                //telemetry.addData("pitch",  mechBot.formatAngle(mechBot.angles.angleUnit, mechBot.angles.thirdAngle));


                //telemetry.addData("grvty",  mechBot.gravity.toString());

               // telemetry.addData("mag",  String.format(Locale.getDefault(), "%.3f",
                //                        Math.sqrt(mechBot.gravity.xAccel*mechBot.gravity.xAccel
               //                                 + mechBot.gravity.yAccel*mechBot.gravity.yAccel
                //                                + mechBot.gravity.zAccel*mechBot.gravity.zAccel)));


                telemetry.addData("","\n");
                telemetry.update();
            }
            else{
                telemetry.addData("Not in sensor or drive mode","");
                telemetry.update();
            }
        }

    }
}
