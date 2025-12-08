package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

public class ControlArm extends RobotOperation{
    ElapsedTime time = new ElapsedTime();
    int currentPhase;
    double maxWait;
    boolean finished;

    public ControlArm() {
        this.maxWait = 1;
    }

    public ControlArm(double maxWait) {
        this.maxWait = maxWait;
    }

    @Override
    public void init(IterativeRobotParent robot) {
        super.init(robot);
        time.reset();
    }

    @Override
    public void loop() {
        if (currentPhase == 0){
            robot.liftLaunchArm();
            currentPhase = 1;
        }
        else if (currentPhase == 1){
            if (time.seconds() > maxWait) {
                currentPhase = 2;
            }
        }
        else if (currentPhase == 2){
            robot.lowerLaunchArm();
            currentPhase = 3;
            time.reset();
        }
        else if (currentPhase == 3){
            if (time.seconds() > maxWait) {
                finished = true;
                currentPhase = 4;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
