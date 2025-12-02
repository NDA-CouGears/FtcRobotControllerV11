package org.firstinspires.ftc.teamcode.operations;

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

    @Override
    public boolean isFinished() {
        return finished;
    }
}
