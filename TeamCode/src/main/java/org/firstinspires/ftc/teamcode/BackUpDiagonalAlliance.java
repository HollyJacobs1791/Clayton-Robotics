package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "BackUpDiagonalAlliance", group = "StarterBot")
public class BackUpDiagonalAlliance extends OpMode {

    private final MecanumDrive drive = new MecanumDrive();
    private final ElapsedTime timer = new ElapsedTime();

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    // Tune these tomorrow
    private static final double BACK_POWER = -0.35;
    private static final double STRAFE_MAG = 0.30; // RED=+, BLUE=-
    private static final double MOVE_TIME  = 0.65;

    private enum State { MOVING, COMPLETE }
    private State state = State.MOVING;

    @Override
    public void init() {
        drive.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Default Alliance", alliance);
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
        timer.reset();
        state = State.MOVING;
    }

    @Override
    public void loop() {
        switch (state) {
            case MOVING:
                double strafe = (alliance == Alliance.RED) ? STRAFE_MAG : -STRAFE_MAG;

                // Back + strafe away from the center line (tune sign if needed)
                drive.drive(BACK_POWER, strafe, 0);

                if (timer.seconds() >= MOVE_TIME) {
                    drive.stop();
                    state = State.COMPLETE;
                }
                break;

            case COMPLETE:
            default:
                drive.stop();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
