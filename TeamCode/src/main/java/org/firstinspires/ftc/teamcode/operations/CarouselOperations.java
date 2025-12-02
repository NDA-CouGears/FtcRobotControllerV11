package org.firstinspires.ftc.teamcode.operations;

import java.util.ArrayList;
import java.util.Arrays;

public class CarouselOperations extends RobotOperation {

    protected boolean finished= false;
    public static ArrayList<String> colors = new ArrayList<String>(Arrays.asList(null, null, null));
    @Override
    public void loop() {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
