package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumFieldRelativeDrive;

/*
 * Autonomous:
 * - Use your Launcher subsystem's state machine to shoot 3 balls
 * - Wait a bit between shots (TIME_BETWEEN_SHOTS)
 * - Back straight up for BACK_UP_TIME
 * - Uses goBILDA StarterBot style (alliance selection in init_loop, enum state machine)
 */

@Autonomous(name = "Auto_Shoot3_BackUp_SM", group = "StarterBot")
public class AutoShoot3BackUp extends OpMode {

    // --- Subsystems ---
    private final Launcher launcher = new Launcher();
    private final MecanumFieldRelativeDrive drive = new MecanumFieldRelativeDrive();

    // --- Robot-level autonomous state machine ---
    private enum AutonomousState {
        LAUNCHING,   // spin up and fire 3 shots
        BACKING_UP,  // drive backwards after shooting
        COMPLETE     // stop and sit still
    }

    private AutonomousState autonomousState;

    // --- Alliance selection, like in StarterBotAuto ---
    private enum Alliance {
        RED,
        BLUE
    }

    private Alliance alliance = Alliance.RED;  // default

    // --- Shot tracking ---
    private int shotsToFire = 3;        // how many TOTAL we want to fire
    private int shotsFired = 0;         // how many we've detected
    private boolean wasLaunching = false;  // for detecting the start of each shot

    // --- Timers ---
    private final ElapsedTime backUpTimer = new ElapsedTime();
    private final ElapsedTime shotTimer   = new ElapsedTime(); // time between shots

    // === Tunable constants ===

    // Back-up behavior (tune these on the field)
    private static final double BACK_UP_TIME  = 1.5;   // seconds to back up
    private static final double BACK_UP_POWER = -0.1;  // throttle (negative = backwards)

    // Time between shots (like TIME_BETWEEN_SHOTS in the sample)
    private static final double TIME_BETWEEN_SHOTS = 1.0;  // seconds between feeds

    // If you want to turn while backing later, change this from 0
    private static final double TURN_MAGNITUDE = 0.0;  // start at 0 = straight

    private boolean started = false;

    // Runs ONCE when you hit INIT
    @Override
    public void init() {
        // Robot-level state machine starts in LAUNCHING, like StarterBotAuto starts in LAUNCH
        autonomousState = AutonomousState.LAUNCHING;

        // Init subsystems
        drive.init(hardwareMap, false);
        launcher.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Default Alliance", alliance);
    }

    // Runs REPEATEDLY between INIT and START
    @Override
    public void init_loop() {
        // Alliance selection (same feel as StarterBotAuto)
        if (gamepad1.b) {
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    // Runs ONCE when you hit START
    @Override
    public void start() {
        autonomousState = AutonomousState.LAUNCHING;
        shotsFired = 0;
        shotsToFire = 3;
        wasLaunching = false;
        backUpTimer.reset();
        shotTimer.reset();
        started = true;
    }

    // Runs REPEATEDLY after START until STOP
    @Override
    public void loop() {
        if (!started) return;

        switch (autonomousState) {

            case LAUNCHING:
                /*
                 * Robot stays still, and we use your Launcher subsystem's state machine
                 * to spin up and feed balls.
                 */
                drive.drive(0, 0, 0);

                // Decide if it's time to request a new shot
                boolean firstShot = (shotsFired == 0);
                boolean timeForNextShot = firstShot || shotTimer.seconds() >= TIME_BETWEEN_SHOTS;

                // Only request a shot when:
                // - Launcher is idle (between feeds)
                // - We haven't already fired all shots
                // - Enough time has passed since the last shot (except for first shot)
                if (launcher.isIdle() && shotsFired < shotsToFire && timeForNextShot) {
                    launcher.startLauncher();   // kicks off SPIN_UP -> LAUNCH -> LAUNCHING internally
                }

                // Once we've fired all shots and we're truly idle, move on
                if (shotsFired >= shotsToFire && launcher.isIdle()) {
                    launcher.stopLauncher();
                    autonomousState = AutonomousState.BACKING_UP;
                    backUpTimer.reset();
                }
                break;

            case BACKING_UP:
                /*
                 * Here we back away from the goal. For now, this is a simple
                 * time-based backup using ArcadeDrive. Later, you can add turning
                 * based on alliance (RED vs BLUE).
                 */

                launcher.stopLauncher();  // make sure launcher stays off

                double turn = 0.0;
                // If you want to turn while backing, you can do:
                if (alliance == Alliance.RED)  turn = TURN_MAGNITUDE;
                else if (alliance == Alliance.BLUE) turn = -TURN_MAGNITUDE;

                drive.drive(BACK_UP_POWER, turn, 0);

                if (backUpTimer.seconds() >= BACK_UP_TIME) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
            default:
                // Robot is done: stop drive and launcher
                drive.drive(0, 0, 0);
                launcher.stopLauncher();
                break;
        }

        // --- Keep the launcher's internal state machine running every loop ---
        launcher.updateState();

        // --- Count shots by watching when LAUNCHING starts (edge detection) ---
        boolean isLaunchingNow = launcher.isLaunching();  // true while feeders are running
        if (isLaunchingNow && !wasLaunching) {
            shotsFired++;
            shotTimer.reset();   // start timing for the next shot
        }
        if (shotsFired > shotsToFire) {
            shotsFired = shotsToFire;
        }
        wasLaunching = isLaunchingNow;

        // --- Telemetry, like StarterBotAuto style ---
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("LauncherState", launcher.getState());
        telemetry.addData("LauncherVelocity", launcher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Safe shutdown
        drive.drive(0, 0, 0);
        launcher.stopLauncher();
    }
}

