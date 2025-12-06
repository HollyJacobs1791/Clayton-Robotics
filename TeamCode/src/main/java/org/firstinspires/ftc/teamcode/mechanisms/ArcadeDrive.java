package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArcadeDrive {
    private DcMotor leftDrive, rightDrive;

    private double SPIN_DAMPING = 2.0; // higher means slower turning

    public void init(HardwareMap hwMap) {
        leftDrive = hwMap.get(DcMotor.class,"left_drive");
        rightDrive = hwMap.get(DcMotor.class,"right_drive");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //NEED TO PLUG MOTOR ENCODERS IN! ^^^

        // reverse left motor
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double throttle, double spin) {
        // apply damping to spin
        spin /= SPIN_DAMPING;

        double leftpower = throttle + spin;
        double rightpower = throttle - spin;
        //clamp and scale values
        double largest = Math.max(Math.abs(throttle), Math.abs(spin));
        if (largest > 1.0) {
            throttle /= largest;
            spin /= largest;
        }

        leftDrive.setPower(leftpower);
        rightDrive.setPower(rightpower);
    }
}

