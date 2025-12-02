package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PrepareLoad extends CarouselOperations {
    int bay;
    public PrepareLoad(int bay){
        this.bay = bay;
    }

    @Override
    public void loop() {
        super.loop();
        if (bay == 1){
            robot.setCarouselPosition(0);
        }
        else if (bay == 2){
            robot.setCarouselPosition(4);
        }
        else if (bay == 3) {
            robot.setCarouselPosition(2);
        }

        if (!robot.isCarouselBusy()){
            finished = true;
        }
    }

}
