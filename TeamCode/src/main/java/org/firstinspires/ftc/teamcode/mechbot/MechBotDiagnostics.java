package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

/**
 * Created by JimLori on 11/22/2017.
 */

@TeleOp (name = "MechBotDiagnostics", group = "Test")

public class MechBotDiagnostics extends LoggingLinearOpMode {

    MechBotSensor bot = new MechBotSensor();


    enum Mode {NORMAL, SINGLE_WHEEL, SINGLE_DIRECTION}

    private Mode mode = Mode.NORMAL;

    private float[] robotXYTheta = new float[]{0, 0, (float)Math.PI/2f};

    private float baseHeading;

    @Override
    public void runLoggingOpmode()  {

        bot.init(hardwareMap);

        waitForStart();

        bot.updateOdometry();

        baseHeading = bot.getHeadingRadians();

        while (opModeIsActive()){

            if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up)
            {
                bot.setDriveSpeed(0,0,0);
                sleep(250);
                baseHeading = bot.getHeadingRadians();
                robotXYTheta = new float[] {0, 0, (float)Math.PI/2f};
                bot.updateOdometry();
                if (gamepad1.dpad_up)mode = Mode.NORMAL;
                else if (gamepad1.dpad_left) mode = Mode.SINGLE_DIRECTION;
                else mode = Mode.SINGLE_WHEEL;
                continue;
            }

            float newHeading = (float) VuMarkNavigator.NormalizeAngle(bot.getHeadingRadians() - baseHeading + (float)Math.PI/2f);
            robotXYTheta = bot.updateOdometry(robotXYTheta, newHeading);
            float[] wheelTicks = bot.last_Wheel_Ticks.getData();

            if (mode == Mode.NORMAL){
                telemetry.addData("Mode", "NORMAL");
                float x = gamepad1.left_stick_x;
                if (Math.abs(x) < 0.05) x = 0;
                float y = -gamepad1.left_stick_y;
                if (Math.abs(y) < 0.05) y = 0;
                float a = gamepad1.right_stick_x;
                if (Math.abs(a) < 0.05) a = 0;
                float vx = x * 40f;
                float vy = y * 40f;
                bot.setDriveSpeed(vx, vy, a);
                telemetry.addData("Speeds"," vx = %.2f vy = %.2f va = %.2f",vx, vy, a);
            }
            else if (mode == Mode.SINGLE_WHEEL){
                telemetry.addData("Mode","SINGLE_WHEEL");
                float v1, v2, v3, v4;
                v1 = gamepad1.x? 0.5f : 0f;
                v2 = gamepad1.y? 0.5f : 0f;
                v3 = gamepad1.b? 0.5f : 0f;
                v4 = gamepad1.a? 0.5f : 0f;
                bot.one.setPower(v1);
                bot.two.setPower(v2);
                bot.three.setPower(v3);
                bot.four.setPower(v4);
            }
            else if (mode == Mode.SINGLE_DIRECTION){
                telemetry.addData("Mode","SINGLE_DIRECTION");
                float vx, vy, va;
                vx = gamepad1.x? 20f : 0f;
                vy = gamepad1.y? 20f : 0f;
                va = gamepad1.a? 0.5f : 0f;
                bot.setDriveSpeed(vx, vy, va);
                telemetry.addData("Speeds"," vx = %.2f vy = %.2f va = %.2f",vx, vy, va);
            }

            telemetry.addData("Heading","%.2f degrees", newHeading * 180.0/Math.PI);
            telemetry.addData("Robot XYTheta","%.2f %.2f %.2f", robotXYTheta[0], robotXYTheta[1], robotXYTheta[2] * 180.0/Math.PI);
            telemetry.addData("Wheel Ticks", "%.0f %.0f %.0f %.0f", wheelTicks[0], wheelTicks[1], wheelTicks[2], wheelTicks[3]);

            telemetry.update();

        }

    }

}
