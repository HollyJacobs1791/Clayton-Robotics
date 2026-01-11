package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "BackUpOnly", group = "StarterBot")
public class BackUpOnly extends OpMode {

    private final MecanumDrive drive = new MecanumDrive();
    private final ElapsedTime timer = new ElapsedTime();

    // Tune these tomorrow on the practice field
    private static final double BACK_UP_POWER = -0.40; // negative = backwards
    private static final double BACK_UP_TIME  = 0.75;  // seconds

    private enum AutoState {
        BACKING_UP,
        COMPLETE
    }

    private AutoState state = AutoState.BACKING_UP;

    @Override
    public void init() {
        drive.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        timer.reset();
        state = AutoState.BACKING_UP;
    }

    @Override
    public void loop() {

        switch (state) {
            case BACKING_UP:
                // Back straight up: forward negative, no strafe, no rotate
                drive.drive(BACK_UP_POWER, 0, 0);

                if (timer.seconds() >= BACK_UP_TIME) {
                    drive.stop();
                    state = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
            default:
                drive.stop();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
