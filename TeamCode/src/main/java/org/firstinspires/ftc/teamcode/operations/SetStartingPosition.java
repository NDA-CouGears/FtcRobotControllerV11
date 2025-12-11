package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

import java.util.Locale;

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

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%2.2f, %2.2f, %2.2f)",getClass().getSimpleName(), x, y, heading));
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
