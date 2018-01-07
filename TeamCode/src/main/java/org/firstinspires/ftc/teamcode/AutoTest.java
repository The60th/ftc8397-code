package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.Intent;
import android.media.MediaPlayer;
import android.net.Uri;
import android.speech.tts.TextToSpeech;
import android.speech.tts.UtteranceProgressListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

import java.util.Locale;

/**
 * Created by FTC Team 8397 on 1/5/2018.
 */
@Autonomous(name = "AutoDemo",group = "Demo")
public class AutoTest extends MechBotAutonomous {
    boolean speaking;
    TextToSpeech Tts = null;

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        Tts = new TextToSpeech(hardwareMap.appContext,new TextToSpeech.OnInitListener(){
            @Override
            public void onInit(int status) {
                if(status == TextToSpeech.SUCCESS){
                    Tts.setLanguage(Locale.US);
                }
            }
        } );

        Tts.setOnUtteranceProgressListener(new UtteranceProgressListener() {
            @Override
            public void onStart(String utteranceId) {

            }

            @Override
            public void onDone(String utteranceId) {
                speaking = false;
            }

            @Override
            public void onError(String utteranceId) {

            }
        });
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a && !speaking){
                speaking = true;
                Tts.speak("Hello",TextToSpeech.QUEUE_FLUSH,null,"Test");
            }
        }
    }
}


