package org.firstinspires.ftc.teamcode.operations;

public class SetShootSpeed extends RobotOperation {
    private final int speed;
    private boolean finished = false;

    public SetShootSpeed(int speed) {
        this.speed = speed;
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
