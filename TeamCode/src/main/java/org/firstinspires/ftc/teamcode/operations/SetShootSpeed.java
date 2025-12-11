package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import java.util.Locale;

public class SetShootSpeed extends RobotOperation {
    private final int speed;
    private boolean finished = false;

    public SetShootSpeed(int speed) {
        this.speed = speed;
    }

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%d):%b",getClass().getSimpleName(), speed, finished));
    }

    @Override
    public void loop() {
        if (!finished) {
            robot.setShootSpeed(speed);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
