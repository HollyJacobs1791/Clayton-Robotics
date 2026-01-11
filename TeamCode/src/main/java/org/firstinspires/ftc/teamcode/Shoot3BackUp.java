package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * Shoot 3, then back up diagonally (alliance-aware).
 * This version waits for the flywheel to reach READY_VELOCITY and hold it for READY_HOLD_TIME
 * before feeding EACH shot, to avoid the 3rd shot being low.
 */
@Autonomous(name = "Shoot3BackUp", group = "StarterBot")
public class Shoot3BackUp extends OpMode {

    // ---- Drive ----
    private final MecanumDrive drive = new MecanumDrive();

    // ---- Launcher hardware (same config names) ----
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    // ---- Alliance selection ----
    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    // ---- Auto state machine ----
    private enum State {
        SPIN_UP,
        HOLD_READY,
        FEEDING,
        BETWEEN_SHOTS,
        BACKING_UP,
        COMPLETE
    }
    private State state = State.SPIN_UP;

    // === Tunables (adjust on the practice field) ===
    private static final int SHOTS_TO_FIRE = 3;

    // Flywheel velocity (SDK velocity units from getVelocity/setVelocity)
    private static final double TARGET_VELOCITY = 1125;

    // Raised so it won't feed shot 3 until the wheel is REALLY back up
    private static final double READY_VELOCITY = 1125;

    // Must stay >= READY_VELOCITY for this long before feeding
    private static final double READY_HOLD_TIME = 0.35;

    // Feeder timing
    private static final double FEED_TIME = 0.23;

    // Give flywheel more recovery time between feeds (big help for shot 3)
    private static final double TIME_BETWEEN_SHOTS = 1.75;

    // Spin-up safety timeout (do NOT feed at low speed; just keep trying)
    private static final double SPINUP_TIMEOUT = 3.0;

    // Feeder power
    private static final double FEED_POWER = 1.0;
    private static final double FEED_STOP  = 0.0;

    // Launcher PIDF (match your Launcher class)
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(300, 0, 0, 10);

    // Backup motion (diagonal to reduce center-line drift risk)
    private static final double BACK_TIME   = 0.70;
    private static final double BACK_POWER  = -0.35;
    private static final double STRAFE_MAG  = 0.28; // RED +, BLUE -

    // ---- Counters & timers ----
    private int shotsFired = 0;

    // stateTimer is used for HOLD_READY, FEEDING, BETWEEN_SHOTS, BACKING_UP
    private final ElapsedTime stateTimer  = new ElapsedTime();

    // spinupTimer is used to avoid getting stuck forever waiting for speed
    private final ElapsedTime spinupTimer = new ElapsedTime();

    private boolean started = false;

    @Override
    public void init() {
        drive.init(hardwareMap);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Inline compliant wheels need opposite rotation together
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        stopFeeders();
        stopLauncher();
        drive.stop();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Default Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) alliance = Alliance.RED;
        else if (gamepad1.x) alliance = Alliance.BLUE;

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void start() {
        started = true;

        shotsFired = 0;
        state = State.SPIN_UP;

        stateTimer.reset();
        spinupTimer.reset();

        stopFeeders();
        drive.stop();

        launcher.setVelocity(TARGET_VELOCITY);
    }

    @Override
    public void loop() {
        if (!started) return;

        double vel = launcher.getVelocity();

        switch (state) {

            case SPIN_UP:
                drive.stop();
                launcher.setVelocity(TARGET_VELOCITY);

                // Start (or continue) the spin-up window
                // NOTE: spinupTimer was reset when entering SPIN_UP from BETWEEN_SHOTS
                // If you ever manually set state = SPIN_UP elsewhere, be sure to reset it too.

                if (vel >= READY_VELOCITY) {
                    state = State.HOLD_READY;
                    stateTimer.reset();
                }

                // If it can't reach READY_VELOCITY in time, DO NOT feed at low speed.
                // Just keep trying (and show it in telemetry).
                if (spinupTimer.seconds() >= SPINUP_TIMEOUT) {
                    spinupTimer.reset();
                }
                break;

            case HOLD_READY:
                drive.stop();
                launcher.setVelocity(TARGET_VELOCITY);

                // If speed dips, go back to SPIN_UP and try again
                if (vel < READY_VELOCITY) {
                    state = State.SPIN_UP;
                    // do NOT reset spinupTimer here; we still want the overall window logic
                    break;
                }

                // Must hold speed for READY_HOLD_TIME before feeding
                if (stateTimer.seconds() >= READY_HOLD_TIME) {
                    state = State.FEEDING;
                    stateTimer.reset();
                    startFeeders();
                }
                break;

            case FEEDING:
                drive.stop();
                launcher.setVelocity(TARGET_VELOCITY);

                if (stateTimer.seconds() >= FEED_TIME) {
                    stopFeeders();
                    shotsFired++;

                    if (shotsFired >= SHOTS_TO_FIRE) {
                        // Done shooting -> backup
                        state = State.BACKING_UP;
                        stateTimer.reset();
                        stopLauncher(); // stop flywheel before moving (optional, but keeps it simple)
                    } else {
                        // Next shot recovery
                        state = State.BETWEEN_SHOTS;
                        stateTimer.reset();
                    }
                }
                break;

            case BETWEEN_SHOTS:
                drive.stop();
                launcher.setVelocity(TARGET_VELOCITY);

                if (stateTimer.seconds() >= TIME_BETWEEN_SHOTS) {
                    state = State.SPIN_UP;
                    stateTimer.reset();
                    spinupTimer.reset(); // fresh window for the next shot
                }
                break;

            case BACKING_UP:
                stopFeeders();
                stopLauncher();

                double strafe = (alliance == Alliance.RED) ? STRAFE_MAG : -STRAFE_MAG;

                // Back + strafe (diagonal)
                drive.drive(BACK_POWER, strafe, 0);

                if (stateTimer.seconds() >= BACK_TIME) {
                    drive.stop();
                    state = State.COMPLETE;
                }
                break;

            case COMPLETE:
            default:
                drive.stop();
                stopFeeders();
                stopLauncher();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("ShotsFired", shotsFired);
        telemetry.addData("Velocity", vel);
        telemetry.addData("SpinupT", spinupTimer.seconds());
        telemetry.addData("StateT", stateTimer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        stopFeeders();
        stopLauncher();
    }

    private void startFeeders() {
        leftFeeder.setPower(FEED_POWER);
        rightFeeder.setPower(FEED_POWER);
    }

    private void stopFeeders() {
        leftFeeder.setPower(FEED_STOP);
        rightFeeder.setPower(FEED_STOP);
    }

    private void stopLauncher() {
        launcher.setVelocity(0);
    }
}
