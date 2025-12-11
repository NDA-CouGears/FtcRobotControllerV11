package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import java.util.Locale;

public class SetIntakeSpeed extends RobotOperation {
    private final int speed;
    private boolean finished = false;

    public SetIntakeSpeed(int speed) {
        this.speed = speed;
    }

    @Override
    public void loop() {
        if (!finished) {
            robot.setIntakeSpeed(speed);
            finished = true;
        }
    }

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%d):%b",getClass().getSimpleName(), speed, finished));
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
