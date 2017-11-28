package org.firstinspires.ftc.teamcode.debug_programs;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
        import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
        import org.firstinspires.ftc.teamcode.mechbot.MechBotSensor;

/**
 * Created by FTC Team 8397 on 10/27/2017.
 */
@Autonomous( name= "Do Almost Nothing", group = "Test")
public class DoAlmostNothing extends MechBotAutonomous {
    @Override
    public void runLoggingOpmode() throws InterruptedException {
            bot.init(hardwareMap,90);
            waitForStart();

            robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};

            driveDirectionGyro(30, 90,90,
                    new Predicate() {
                        @Override
                        public boolean isTrue() {
                            return robotZXPhi[1] > 100;
                        }
                    });

    }
}
