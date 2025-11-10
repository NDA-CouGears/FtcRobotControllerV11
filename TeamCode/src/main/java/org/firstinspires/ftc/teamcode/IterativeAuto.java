package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "IterativeAuto", group = "Tournament")

public class IterativeAuto extends IterativeRobotParent {
    List<RobotOperation> operations = new ArrayList<RobotOperation>();
    int curOperation = -1; //start at -1 so we initialize the first operation first time through loop

    @Override
    public void init() {
        initHardware();
        initAprilTag();
        setCurrentPosition(55, -15, -90);
        operations.add(new IterativeDriveToLocation(0.6, 10, -20, 45));

    }

    @Override
    public void loop() {
        if (curOperation == -1){
            curOperation++;
            RobotOperation robotCurOperation = operations.get(curOperation);
            robotCurOperation.init(this);

        }
        RobotOperation robotCurOperation = operations.get(curOperation);
        robotCurOperation.loop();
        if (robotCurOperation.isFinished() && (operations.size() - 1) > curOperation){
            curOperation++;
            robotCurOperation = operations.get(curOperation);
            robotCurOperation.init(this);
        }
    }

}
