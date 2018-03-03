package org.firstinspires.ftc.teamcode.mechbot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.presupers_bot.MechBotRedHook;

/**
 * Created by FTC Team 8397 on 12/19/2017.
 */

public class MechBotTeleop extends LoggingLinearOpMode {
    public MechBotRedHook bot = new MechBotRedHook();
    public enum State{NORMAL,QUAD_DRIVE,CUSTOM_DRIVE,SINGLE_CONTROLLER_DRIVE}
    public State state = State.NORMAL;
    private float initHeadingRadians;
    private float initPitchRadians;
    private float initRollRadians;
    CustomDrive customDrive = null;



    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);
        resetInitImuValues();

        waitForStart();
        while (opModeIsActive()){
            switch (state){
                case NORMAL:
                    if(gamepad1.x){
                        //customDrive = new CustomDrive() {}
                        state = state.CUSTOM_DRIVE;
                    } else if(gamepad1.b){
                        state = state.QUAD_DRIVE;
                    }else{
                        normalDrive();
                    }
                    break;
                case QUAD_DRIVE:
                    break;
                case CUSTOM_DRIVE:
                    if(!gamepad1.x) break;

                    if(customDrive !=null)customDrive.DriveOneCycle();
                    break;
                case SINGLE_CONTROLLER_DRIVE:
                    break;
                default:
                    break;
            }
        }
    }




    public void resetInitImuValues(){
        Orientation angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        this.initHeadingRadians = angles.firstAngle;
        this.initRollRadians = angles.secondAngle;
        this.initPitchRadians = angles.thirdAngle;

    }

    public interface CustomDrive{
        void DriveOneCycle();
    }
    public void normalDrive(){
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
        bot.setDrivePower((-y), (-x), (-a));
    }



}
