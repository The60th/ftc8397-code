package org.firstinspires.ftc.teamcode.competition_in_work.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

/**
 * Created by FTC Team 8397 on 12/1/2017.
 */
@Autonomous(name = "Auto Pos Balance",group = "Testing")
public class AutonomousAutoBalanceStonePos extends MechBotAutonomous {
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);
        initAuto(TeamColor.RED,2000,2000);
        ElapsedTime et = new ElapsedTime();
        startingAutoPos(5000);

        while (opModeIsActive()) {
            telemetry.addData("ET ", et.milliseconds());
            OpenGLMatrix robotPose = VuMarkNavigator.getRobotPoseRelativeToTarget();
            if (robotPose == null) telemetry.addData("", "Robot Pose Unknown");
            else {
                float[] poseData = robotPose.getData();
                float heading = (float) Math.atan2(poseData[8], poseData[10]) * 180.0f / (float) Math.PI;
                telemetry.addData("Robot Pose", " Heading = %.1f Z = %.1f X = %.1f", heading, poseData[14] / 10.0, poseData[12] / 10.0);
            }
            telemetry.update();
        }
    }
}
