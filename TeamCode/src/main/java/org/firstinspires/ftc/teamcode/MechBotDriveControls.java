package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

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
        mechBot.setDrivePower(((x/1.25f)/this.speedScaler), (-y/this.speedScaler), (-a/this.speedScaler));
        return true;
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
