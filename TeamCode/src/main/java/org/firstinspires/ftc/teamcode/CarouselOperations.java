package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class CarouselOperations extends RobotOperation{

    protected static final float CTR = 751.8f;
    protected float curPos = 0;
    protected float offset;
    protected float zero;
    protected float targetPos;
    protected boolean finished= false;
    ArrayList<String> colors = new ArrayList<String>();
    @Override
    public void loop() {
        offset = curPos % CTR;
        zero = curPos - offset;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
