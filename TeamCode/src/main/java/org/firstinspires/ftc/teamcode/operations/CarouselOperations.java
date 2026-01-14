package org.firstinspires.ftc.teamcode.operations;

import java.util.ArrayList;
import java.util.Arrays;

public class CarouselOperations extends RobotOperation {

    protected boolean finished= false;
    public static final int LAUNCH1 = 0;
    public static final int LOAD3 = 1;
    public static final int LAUNCH2 = 2;
    public static final int LOAD1 = 3;
    public static final int LAUNCH3 = 4;
    public static final int LOAD2 = 5;

    public static ArrayList<String> colors = new ArrayList<String>(Arrays.asList(null, null, null));

    public static void resetColors() {
        colors = new ArrayList<String>(Arrays.asList(null, null, null));
    }

    @Override
    public void loop() {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
