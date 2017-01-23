package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Jim on 11/11/2016.
 */
@Autonomous(name="Test Omni Debug", group="Debug")
@Disabled
public class OmniBotDebugOpMode extends LinearOpMode {

    OmniBotDEBUG robot = new OmniBotDEBUG();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        boolean awaitingButtonRelease = false;
        int maxSpeed = 4000;
        float vx = 0;
        float vy = 0;
        float va = 0;
        float totalTime = 4000;

        while (!gamepad1.start) {
            if (awaitingButtonRelease){
                if (!gamepad1.dpad_up && !gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
                    telemetry.addData("Max Speed", " %d", maxSpeed);
                    telemetry.addData("Req Speed", " vx = %.0f vy = %.0f va = %.0f", vx, vy, va);
                    telemetry.addData("Total Time", " %.0f", totalTime);
                    telemetry.addData("Press Gamepad Start to Continue","");
                    telemetry.update();
                    awaitingButtonRelease = false;
                }
            }
            else if (gamepad1.dpad_up){
                if (maxSpeed == 4000) maxSpeed = 1000;
                else maxSpeed += 500;
                awaitingButtonRelease = true;
            }
            else if (gamepad1.a){
                if (va >= 359.99) va = -360;
                else va += 30;
                awaitingButtonRelease = true;
            }
            else if (gamepad1.x){
                if (vx >= 59.99) vx = -60;
                else vx += 20;
                awaitingButtonRelease = true;
            }
            else if (gamepad1.y){
                if (vy >= 59.99) vy = -60;
                else vy += 20;
                awaitingButtonRelease = true;
            }
            else if (gamepad1.b){
                if (totalTime >= 4999) totalTime = 1000;
                else totalTime += 500;
                awaitingButtonRelease = true;
            }

            idle();
        }

        robot.setMaxDriveTicksPerSec(maxSpeed);

        waitForStart();

        int initialOneTicks = robot.one.getCurrentPosition();
        int initialTwoTicks = robot.two.getCurrentPosition();
        int initialThreeTicks = robot.three.getCurrentPosition();
        int initialFourTicks = robot.four.getCurrentPosition();

        int oldOneTicks = initialOneTicks;
        int oldTwoTicks = initialTwoTicks;
        int oldThreeTicks = initialThreeTicks;
        int oldFourTicks = initialFourTicks;

        robot.setDriveSpeed(vx, vy, va * Math.PI/180.0);

        ElapsedTime totalTimer = new ElapsedTime();

        int oneTicks, twoTicks, threeTicks, fourTicks;
        double oneTicksPerSec, twoTicksPerSec, threeTicksPerSec, fourTicksPerSec;
        double onePower = robot.one.getPower();
        double twoPower = robot.two.getPower();
        double threePower = robot.three.getPower();
        double fourPower = robot.four.getPower();
        double reqOneTicksPerSec = onePower * (double)robot.one.getMaxSpeed();
        double reqTwoTicksPerSec = twoPower * (double)robot.two.getMaxSpeed();
        double reqThreeTicksPerSec = threePower * (double)robot.three.getMaxSpeed();
        double reqFourTicksPerSec = fourPower * (double)robot.four.getMaxSpeed();

//       while (opModeIsActive() && totalTimer.milliseconds() < totalTime){
//            double intervalSec = intervalTimer.seconds();
//            if (intervalSec > 200){
//                oneTicks = robot.one.getCurrentPosition();
//                twoTicks = robot.two.getCurrentPosition();
//                threeTicks = robot.three.getCurrentPosition();
//                fourTicks = robot.four.getCurrentPosition();
//                oneTicksPerSec = (double)(oneTicks - oldOneTicks)/intervalSec;
//                twoTicksPerSec = (double)(twoTicks - oldTwoTicks)/intervalSec;
//                threeTicksPerSec = (double)(threeTicks - oldThreeTicks)/intervalSec;
//                fourTicksPerSec = (double)(fourTicks - oldFourTicks)/intervalSec;
//                oldOneTicks = oneTicks;
//                oldTwoTicks = twoTicks;
//                oldThreeTicks = threeTicks;
//                oldFourTicks = fourTicks;
//                DbgLog.msg("Motor Powers: %.2f %.2f %.2f %.2f", onePower, twoPower, threePower, fourPower);
//                DbgLog.msg("Requested Ticks/Sec: %.0f %.0f %.0f %.0f", reqOneTicksPerSec, reqTwoTicksPerSec,
//                        reqThreeTicksPerSec, reqFourTicksPerSec);
//                DbgLog.msg("Actual Ticks/Sec: %.0f %.0f %.0f %.0f", oneTicksPerSec, twoTicksPerSec,
//                        threeTicksPerSec, fourTicksPerSec);
//                intervalTimer.reset();
//            }
//            idle();
//        }
        sleep((long)totalTime);
        double totalSec = totalTimer.seconds();
        oneTicks = robot.one.getCurrentPosition();
        twoTicks = robot.two.getCurrentPosition();
        threeTicks = robot.three.getCurrentPosition();
        fourTicks = robot.four.getCurrentPosition();
        oneTicksPerSec = (double)(oneTicks - initialOneTicks)/totalSec;
        twoTicksPerSec = (double)(twoTicks - initialTwoTicks)/totalSec;
        threeTicksPerSec = (double)(threeTicks - initialThreeTicks)/totalSec;
        fourTicksPerSec = (double)(fourTicks - initialFourTicks)/totalSec;
        DbgLog.msg("New debug data: Newer");
        DbgLog.msg("Final Ticks: %d %d %d %d", oneTicks- initialOneTicks, twoTicks- initialTwoTicks, threeTicks- initialThreeTicks, fourTicks- initialFourTicks);
        DbgLog.msg("Avg Ticks/Sec: %.0f %.0f %.0f %.0f", oneTicksPerSec, twoTicksPerSec, threeTicksPerSec,
                fourTicksPerSec);

        robot.setDrivePower(0,0,0);

        telemetry.addData("Motor Power", " %.2f %.2f %.2f %.2f", onePower, twoPower, threePower,
                fourPower);
        telemetry.addData("Max Speeds", " %d %d %d %d", robot.one.getMaxSpeed(), robot.two.getMaxSpeed(),
                robot.three.getMaxSpeed(), robot.four.getMaxSpeed());
        telemetry.addData("Elapsed Time", " %.2f", totalSec);
        telemetry.addData("Total Ticks", " %d %d %d %d", oneTicks, twoTicks, threeTicks, fourTicks);
        telemetry.addData("Req Ticks/Sec", " %.0f %.0f %.0f %.0f", reqOneTicksPerSec, reqTwoTicksPerSec,
                reqThreeTicksPerSec, reqFourTicksPerSec);
        telemetry.addData("Actual Ticks/Sec", " %.0f %.0f %.0f %.0f", oneTicksPerSec, twoTicksPerSec,
                threeTicksPerSec, fourTicksPerSec);
        telemetry.update();

        while (opModeIsActive()) idle();


    }

}
