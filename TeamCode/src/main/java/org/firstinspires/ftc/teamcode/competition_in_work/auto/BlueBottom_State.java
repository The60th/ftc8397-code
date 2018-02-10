package org.firstinspires.ftc.teamcode.competition_in_work.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 1/18/2018.
 */

//Work on this later.

public class BlueBottom_State extends MechBotAutonomous {
    private enum State{PRE_INIT,INIT,DRIVING_OFF_STONE,DRIVING_TO_TRIANGLE,LINE_FOLLOW, ADJUST_POSITION,CRYPTO_BOX_SHIFT,SCORING_GLYPH}
    private State state = State.PRE_INIT;
    final boolean BLUE_BOTTOM_START_LOG = false;
    final String BLUE_BOTTOM_START_TAG = "Blue bottom start Red Hook:";
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        switch (state){
            case PRE_INIT:
                telemetry.addData("State: " + state.toString().toLowerCase(), "");
                bot.init(hardwareMap,180); //Init the hardware map with a starting angle of 0.
                //The starting angle is the gyro heading relative to the crypto box.
                robotZXPhi = new float[3];
                state = State.INIT;
                telemetry.update();
                telemetry.addAction(new Runnable() {
                    @Override
                    public void run() {

                    }
                });
                break;
            case INIT:
                telemetry.addData("State: " + state.toString().toLowerCase(), "");
                if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "INITIALIZE AUTO");
                initAuto(TeamColor.BLUE, VUMARK_KEY_SCAN_TIME,JEWEL_SCAN_TIME); //Find the targetJewl side and the target crypto key.
                break;
            case DRIVING_OFF_STONE:
                telemetry.addData("State: " + state.toString().toLowerCase(), "");
                break;
            case DRIVING_TO_TRIANGLE:
                telemetry.addData("State: " + state.toString().toLowerCase(), "");
                break;
            case LINE_FOLLOW:
                telemetry.addData("State: " + state.toString().toLowerCase(), "");
                break;
            case ADJUST_POSITION:
                telemetry.addData("State: " + state.toString().toLowerCase(), "");
                break;
            case CRYPTO_BOX_SHIFT:
                telemetry.addData("State: " + state.toString().toLowerCase(), "");
                break;
            case SCORING_GLYPH:
                telemetry.addData("State: " + state.toString().toLowerCase(), "");
                break;
            default:
                telemetry.addData("State: " + "Failed", "");
                break;
        }
    }
}
