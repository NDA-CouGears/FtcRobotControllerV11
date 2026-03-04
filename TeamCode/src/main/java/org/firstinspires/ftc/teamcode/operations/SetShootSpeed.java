package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import java.util.Locale;

public class SetShootSpeed extends RobotOperation {
    private final double speed;
    private boolean finished = false;

    public SetShootSpeed(double speed) {
        this.speed = speed;
    }

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%f):%b",getClass().getSimpleName(), speed, finished));
    }

    @Override
    public void loop() {
        if (!finished) {
            robot.setShootSpeedVar(speed);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
