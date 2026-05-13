package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

public class IterativeDriveToBall extends RobotOperation {
    boolean finished = false;
    @Override
    public void loop() {
        if (finished) return;

        double[] errors = robot.getBallErrors();
        double distance = errors[0];
        double headingError = errors[1];

        double turnSpeed = Range.clip(headingError * .02, -1, 1);
        double ySpeed = 0;
        if (distance > 0.1) {
            ySpeed = Range.clip(distance * IterativeRobotParent.P_DRIVE_GAIN, 0.1, 1);
        }

        if (Math.abs(headingError) > 1) {
            robot.moveRobot(0,0,turnSpeed);
        }
        /* else if (Math.abs(distance) > 24) {
            robot.moveRobot(0,ySpeed,0);
        }
         */
        else {
            robot.moveRobot(0,0,0);
            //finished = true;
        }

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
