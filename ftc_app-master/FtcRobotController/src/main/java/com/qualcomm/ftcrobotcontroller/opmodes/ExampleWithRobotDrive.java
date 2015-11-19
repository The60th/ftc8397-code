package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class ExampleWithRobotDrive extends OpMode {
    RobotDrive myRobotDrive;

    @Override
    public void init() {
      myRobotDrive = new RobotDrive(hardwareMap.dcMotor.get("left_drive"),
                                    hardwareMap.dcMotor.get("right_drive"));
    }

    @Override
    public void loop() {
      // Call the arcadeDrive method of the RobotDrive class
      myRobotDrive.arcadeDrive(gamepad1);
    }
}
