package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

public class ControlArm extends RobotOperation{
    ElapsedTime time = new ElapsedTime();
    int currentPhase;
    @Override
    public void init(IterativeRobotParent robot) {
        super.init(robot);
        time.reset();
    }

    @Override
    public void loop() {
        if (currentPhase == 0){
            
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
