package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotSensor;

import java.util.ArrayList;

/**
 * Created by JimLori on 11/21/2017.
 */

@Autonomous(name = "GetMotorInfo", group = "Test")
@Disabled

public class GetMotorInfo extends LinearOpMode {

    MechBot bot = new MechBotSensor();

    @Override
    public void runOpMode()  {

        bot.init(hardwareMap);

        DcMotor[] motors = new DcMotor[]{bot.one, bot.two, bot.three, bot.four};
        for (int motor = 0; motor<4; motor++){
            DcMotorController controller = motors[motor].getController();
            MotorConfigurationType mType = controller.getMotorType(motor);
            double ticksPerRev = mType.getTicksPerRev();
            double gearing = mType.getGearing();
            double maxRPM = mType.getMaxRPM();
            double achievableMaxRPMFraction = mType.getAchieveableMaxRPMFraction();

            telemetry.addData("Motor","%d: tpr=%.0f gr=%.0f RPM=%.0f Frac=%.2f", motor, ticksPerRev, gearing,
                    maxRPM, achievableMaxRPMFraction);
        }

        telemetry.update();

        waitForStart();

    }

}
