package org.firstinspires.ftc.teamcode;

public abstract class RobotOperation {
    protected IterativeRobotParent robot;
    public void init(IterativeRobotParent robot){
        this.robot = robot;
    }

    public void loop(){}
    public boolean isFinished(){
        return true;
    }

    public void stop(){}
}
