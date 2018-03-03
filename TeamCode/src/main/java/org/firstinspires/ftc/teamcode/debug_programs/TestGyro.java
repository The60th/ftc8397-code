package org.firstinspires.ftc.teamcode.debug_programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mechbot.presupers_bot.MechBotSensor;

/**
 * Created by FTC Team 8397 on 11/4/2017.
 */
@Autonomous(name="TestGyroDrive", group="Rev")
@Disabled
public class TestGyro extends LinearOpMode {
    MechBotSensor bot = new MechBotSensor();
    Orientation orientation;
    private final byte AXIS_MAP_CONFIG_BYTE = 0x6;
    private final byte AXIS_MAP_SIGN_BYTE = 0x1;
    @Override
    public void runOpMode() throws InterruptedException { //DQ16P092
        bot.init(hardwareMap,0);

       // bot.imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100);

       // bot.imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
       // bot.imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

       // bot.imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);
        waitForStart();
        while (opModeIsActive()) {
            orientation = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //WAS ZYX

            telemetry.addData("Heading degrees: ",orientation.firstAngle);
            telemetry.addData("Roll degrees: ",orientation.secondAngle);
            telemetry.addData("Pitch degrees: ", orientation.thirdAngle);

            //orientation = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, A)
            telemetry.update();
        }
    }
}
