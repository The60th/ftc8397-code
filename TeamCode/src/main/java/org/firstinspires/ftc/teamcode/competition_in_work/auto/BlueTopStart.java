package org.firstinspires.ftc.teamcode.competition_in_work.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

/**
 * Created by FTC Team 8397 on 11/22/2017.
 */
@Autonomous( name= "BlueTop", group = "Auto")
public class BlueTopStart extends MechBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {

        //Call this initAuto method passing in teamcolor and our cv timeout values.
        //After this we can use the variables of cryptokey and target side to figure out what to do next.
        //cryptoKey will contain the part of the cryptobox we want to go to, while the targetSide is what jewel we want to knockoff.
        initAuto(TeamColor.BLUE,1500,1500); //This will pause the program till it has start, it contains a wait for start.

        telemetry.addData("Auto data: ","Vumark target: " + cryptoKey + " target jewel side: " + targetSide);
        telemetry.update();

    }
}
