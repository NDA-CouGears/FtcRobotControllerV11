package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

public class PrepareLoad extends CarouselOperations {
    int bay;
    boolean posSet;
    public PrepareLoad(int bay){
        this.bay = bay;
    }

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%d):%b",getClass().getSimpleName(), bay, posSet));
    }

    @Override
    public void loop() {
        super.loop();
        if (!posSet) {
            posSet = true;
            if (bay == 1) {
                robot.setCarouselPosition(LOAD1);
            } else if (bay == 2) {
                robot.setCarouselPosition(LOAD2);
            } else if (bay == 3) {
                robot.setCarouselPosition(LOAD3);
            }
        }

        if (!robot.isCarouselBusy()){
            finished = true;
        }
    }

}
