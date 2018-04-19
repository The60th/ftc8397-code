package org.firstinspires.ftc.teamcode.competition_in_work.driver_control;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.cv_programs.Blob;
import org.firstinspires.ftc.teamcode.cv_programs.ImgProc;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

/**
 * Created by FTC Team 8397 on 4/18/2018.
 */
@TeleOp(name = "PreMatchRobotAlignment--No Vuforia", group = "Debug")
public class SensorDebug extends LoggingLinearOpMode {
    Orientation orientation;
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        MechBotScranton bot = new MechBotScranton();
        bot.init(hardwareMap);
        waitForStart();
        telemetry.addData("Program started. ", "");
        telemetry.update();

        while (opModeIsActive()) {

            orientation = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //WAS ZYX
            telemetry.addData("Gyro Heading degrees: ",orientation.firstAngle);
            telemetry.addData("Roll degrees: ",orientation.secondAngle);
            telemetry.addData("Pitch degrees: ", orientation.thirdAngle);

            float[] HSV = new float[3];
            Color.RGBToHSV(bot.colorLeft.red(), bot.colorLeft.green(), bot.colorLeft.blue(),HSV);
            telemetry.addData("Color left", "H = %.2f S = %.2f V = %.2f", HSV[0],HSV[1],HSV[2]);

            Color.RGBToHSV(bot.colorRight.red(), bot.colorRight.green(), bot.colorRight.blue(),HSV);
            telemetry.addData("Color right", "H = %.2f S = %.2f V = %.2f", HSV[0],HSV[1],HSV[2]);

            Color.RGBToHSV(bot.backColorLeft.red(), bot.backColorLeft.green(), bot.backColorLeft.blue(),HSV);
            telemetry.addData("Color back left", "H = %.2f S = %.2f V = %.2f", HSV[0],HSV[1],HSV[2]);

            Color.RGBToHSV(bot.backColorRight.red(), bot.backColorRight.green(), bot.backColorRight.blue(),HSV);
            telemetry.addData("Color right", "H = %.2f S = %.2f V = %.2f", HSV[0],HSV[1],HSV[2]);

            boolean touch = bot.touchSensor.getState();
            telemetry.addData("touch State", touch);
            telemetry.addData("flip plate ticks", bot.flipMotor.getCurrentPosition());
            telemetry.addData("Glyph color: ", bot.getIntakeColor());
            telemetry.addData("ODS Distance (CM):", bot.getIntakeDistance());

            telemetry.update();

        }
    }
}
