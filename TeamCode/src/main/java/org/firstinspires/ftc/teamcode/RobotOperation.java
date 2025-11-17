package org.firstinspires.ftc.teamcode;

public abstract class RobotOperation {
    protected IterativeRobotParent robot;
    public void init(IterativeRobotParent robot){
        this.robot = robot;
    }

    public abstract void loop();
    public abstract boolean isFinished();

    public void stop(){}
}
