package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PrepareLaunch extends CarouselOperations {
    int bay;
    boolean posSet;

    public PrepareLaunch(int bay){
        this.bay = bay;
    }

    @Override
    public void loop() {
        super.loop();
        if (!posSet) {
            posSet = true;
            if (bay == 1) {
                robot.setCarouselPosition(3);
            } else if (bay == 2) {
                robot.setCarouselPosition(5);
            } else if (bay == 3) {
                robot.setCarouselPosition(1);
            }
        }

        if (!robot.isCarouselBusy()) {
            finished = true;
        }

    }
}
