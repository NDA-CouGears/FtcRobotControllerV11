package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

public class SetStartingPosition extends RobotOperation{
    double x;
    double y;
    double heading;
    public SetStartingPosition(double x, double y, double heading, boolean isRed){
        if (!isRed) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
        else {
            this.x = x;
            this.y = -y;
            this.heading = 180 - heading;
        }
    }

    @Override
    public void init(IterativeRobotParent robot) {
        super.init(robot);
        robot.setCurrentPosition(x, y, heading);
    }

    @Override
    public void loop() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
