package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumFieldRelativeDrive;

public class Teleop extends OpMode {
    MecanumFieldRelativeDrive drive = new MecanumFieldRelativeDrive();
    double forward, strafe, rotate;
    Launcher launcher = new Launcher();
    @Override
    public void init() {
        drive.init(hardwareMap, false);
        launcher.init(hardwareMap);
    }
    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);

        if (gamepad1.y) {
            launcher.startLauncher();
        }
        else if (gamepad1.b) {
            launcher.stopLauncher();
        }

        //update state machine
        launcher.updateState();

        telemetry.addData("State", launcher.getState());
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
    }
}

