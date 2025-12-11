package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

import java.util.Locale;

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

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%2.2f)",getClass().getSimpleName(), time));
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return timer.seconds() >= time;
    }
}
