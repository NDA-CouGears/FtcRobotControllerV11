package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Sleep extends RobotOperation{
    double time;
    ElapsedTime timer = new ElapsedTime();
    public Sleep(double time){
        this.time = time;
    }

    @Override
    public void init(IterativeRobotParent robot){
        super.init(robot);
        timer.reset();
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return timer.seconds() >= time;
    }
}
