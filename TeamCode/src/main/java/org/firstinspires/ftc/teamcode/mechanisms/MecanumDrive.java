package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // Optional: store last powers for telemetry/debug
    private double leftFrontPower  = 0.0;
    private double rightFrontPower = 0.0;
    private double leftBackPower   = 0.0;
    private double rightBackPower  = 0.0;

    public void init(HardwareMap hardwareMap) {
        // Must match Robot Configuration names exactly
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Match StarterBotTeleOp directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Match StarterBotTeleOp braking behavior
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // Start stopped
        stop();
    }

    /**
     * Drive using the same mixing as StarterBotTeleOp:
     * forward: + forward, - backward
     * strafe:  + right,   - left
     * rotate:  + clockwise, - counterclockwise (depends on your sign convention)
     */
    public void drive(double forward, double strafe, double rotate) {
        // Denominator is the largest motor power (absolute value) or 1.
        // This keeps ratios the same while ensuring values stay within [-1, 1].
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);

        leftFrontPower  = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower   = (forward - strafe + rotate) / denominator;
        rightBackPower  = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void stop() {
        if (leftFrontDrive == null) return; // if called before init
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // Optional getters (nice for telemetry)
    public double getLeftFrontPower()  { return leftFrontPower; }
    public double getRightFrontPower() { return rightFrontPower; }
    public double getLeftBackPower()   { return leftBackPower; }
    public double getRightBackPower()  { return rightBackPower; }
}
