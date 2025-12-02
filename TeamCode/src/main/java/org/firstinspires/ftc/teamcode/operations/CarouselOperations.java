package org.firstinspires.ftc.teamcode.operations;

import java.util.ArrayList;
import java.util.Arrays;

public class CarouselOperations extends RobotOperation {

    protected static final float CTR = 751.8f;
    protected float curPos;
    protected float offset;
    protected float zero;
    protected float targetPos;
    protected boolean finished= false;
    public static ArrayList<String> colors = new ArrayList<String>(Arrays.asList(null, null, null));
    @Override
    public void loop() {
        curPos = robot.carousel.getCurrentPosition();
        offset = curPos % CTR;
        zero = curPos - offset;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
