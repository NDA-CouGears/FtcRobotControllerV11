package org.firstinspires.ftc.teamcode;

public abstract class RobotOperation {
    protected RobotParent robot;
    public void init(RobotParent robot){
        this.robot = robot;
    }

    public void loop(){}
    public boolean isFinished(){
        return true;
    }

    public void stop(){}
}
