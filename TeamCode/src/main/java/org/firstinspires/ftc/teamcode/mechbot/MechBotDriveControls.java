package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 10/6/2017.
 */

public class MechBotDriveControls {
    //Problem with mechBotSensor not being an subclass and rather its own true class.
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private MechBot mechBot;
    private float speedScaler = 1;
    private boolean gamepadRefreshed = false;
    public static enum XYZ {plusX,plusY,plusZ,negX,negY,negZ};

    UTILToggle turnSlowModeToggle = new UTILToggle();
    boolean turnSlowMode = false;

    public MechBotDriveControls(Gamepad gamepad1, Gamepad gamepad2, MechBot mechBot){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.mechBot = mechBot;
    }
    public MechBotDriveControls(Gamepad gamepad1, Gamepad gamepad2, MechBot mechBot, float speedScaler){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.mechBot = mechBot;
        this.speedScaler = speedScaler;
    }
    public boolean refreshGamepads(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepadRefreshed = true;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        return true;
    }
    public boolean joyStickMecnumDrive(){
        if(!this.gamepadRefreshed){
            return false;
        }
        float x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
        float y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        float a = 0;
        if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
            if (gamepad1.left_trigger > gamepad1.right_trigger)
                a = -gamepad1.left_trigger;
            else {
                a = gamepad1.right_trigger;
            }
        }
        mechBot.setDrivePower((x/this.speedScaler), (-y/this.speedScaler), (-a/this.speedScaler));
        return true;
    }
    //OLD
    public void joyStickMecnumDriveComp(float[] data){
        float scaler = 1.0f;
        if(!this.gamepadRefreshed){
            data = null;
        }
        float x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
        float y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        float a = 0;
        if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
            if (gamepad1.left_trigger > gamepad1.right_trigger)
                a = -gamepad1.left_trigger;
            else {
                a = gamepad1.right_trigger;
            }
        }
        mechBot.setDrivePower((-y/scaler), (-x/scaler), (-a/scaler));
        data[0] = y;
        data[1] = x;
        data[2] = a;
        data[3] = y/scaler;
        data[4] = x/scaler;
        data[5] = a/scaler;

    }

    public void joyStickMecnumDriveCompQuadSlow(float[] data){
        float quadFactor = 1.75f;
        if(!this.gamepadRefreshed){
            data = null;
        }
        if(turnSlowModeToggle.status(gamepad1.dpad_left) == UTILToggle.Status.COMPLETE){
            if(!turnSlowMode){
                turnSlowMode = true;
            }else{
                turnSlowMode = false;
            }
        }
        float x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
        float y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        float a = 0;
        if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
            if (gamepad1.left_trigger > gamepad1.right_trigger)
                a = -gamepad1.left_trigger;
            else {
                a = gamepad1.right_trigger;
            }
        }
        float modifer = 1.0f;
        if(turnSlowMode){
            modifer = 2.0f;
        }
        mechBot.setDrivePower(
                (-(Math.signum(y)*(Math.pow(Math.abs(y),quadFactor))))*.800f, //Was .70f
                (-(Math.signum(x)*(Math.pow(Math.abs(x),quadFactor))))*.800f,
                (-(Math.signum(a)*(Math.pow(Math.abs(a),quadFactor))))*.800f/modifer
        );

        data[0] = y;
        data[1] = x;
        data[2] = a;
        data[3] = (float)Math.pow(y,quadFactor)*.800f;
        data[4] = (float)Math.pow(x,quadFactor)*.800f;
        data[5] = (float)Math.pow(a,quadFactor)*.800f;

    }
    public void joyStickMecnumDriveCompQuad(float[] data){
        float quadFactor = 1.75f;
        if(!this.gamepadRefreshed){
            data = null;
        }
        float x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
        float y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        float a = 0;
        if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
            if (gamepad1.left_trigger > gamepad1.right_trigger)
                a = -gamepad1.left_trigger;
            else {
                a = gamepad1.right_trigger;
            }
        }
        mechBot.setDrivePower(
                (-(Math.signum(y)*(Math.pow(Math.abs(y),quadFactor)))),
                (-(Math.signum(x)*(Math.pow(Math.abs(x),quadFactor)))),
                (-(Math.signum(a)*(Math.pow(Math.abs(a),quadFactor))))
        );
        data[0] = y;
        data[1] = x;
        data[2] = a;
        data[3] = (float)Math.pow(y,quadFactor);
        data[4] = (float)Math.pow(x,quadFactor);
        data[5] = (float)Math.pow(a,quadFactor);

    }
    public boolean joyStickMecnumDrive(float speedScaler){
        if(!this.gamepadRefreshed){
            return false;
        }
        float x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
        float y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        float a = 0;
        if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
            if (gamepad1.left_trigger > gamepad1.right_trigger)
                a = -gamepad1.left_trigger;
            else {
                a = gamepad1.right_trigger;
            }
        }
        mechBot.setDrivePower((x/speedScaler), (y/speedScaler), (a/speedScaler));
        return true;
    }
    public boolean driveDirectonByPower(XYZ xyz, double speed){
        if(!this.gamepadRefreshed){
            return false;
        }
        if(xyz == XYZ.plusX){mechBot.setDrivePower(speed,0,0);}
        else if(xyz == XYZ.plusY){mechBot.setDrivePower(0,speed,0);}

        else if(xyz == XYZ.negX){mechBot.setDrivePower(speed,0,0);}
        else if(xyz == XYZ.negY){mechBot.setDrivePower(0,speed,0);}
        return false;
    }
    public boolean tankDrive(){
        return true;
    }
    public boolean isGamepadRefreshed(){
        return gamepadRefreshed;
    }








    public Gamepad getGamepad1() {
        return this.gamepad1;
    }

    public Gamepad getGamepad2() {
        return this.gamepad2;
    }

    public MechBot getMechBot() {
        return this.mechBot;
    }

}
