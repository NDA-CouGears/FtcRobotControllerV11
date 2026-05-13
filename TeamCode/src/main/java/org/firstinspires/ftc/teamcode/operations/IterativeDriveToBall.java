package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.util.Range;

public class IterativeDriveToBall extends RobotOperation {
    boolean finished = false;
    @Override
    public void loop() {
        if (finished) return;

        double[] errors = robot.getBallErrors();
        double distance = errors[0];
        double headingError = errors[1];

        double turnSpeed = Range.clip(headingError * .02, -1, 1);
        if (Math.abs(headingError) > 1) {
            robot.moveRobot(0,0,turnSpeed);
        }
        else {
            robot.moveRobot(0,0,0);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
