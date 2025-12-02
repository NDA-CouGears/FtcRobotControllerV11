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
            targetPos = zero;
        }
        else if (bay == 2){
            targetPos = zero + (2*CTR/3);
        }
        else if (bay == 3) {
            targetPos = zero + (CTR / 3);
        }
        if (targetPos < curPos){
            targetPos += CTR;
        }

        robot.carousel.setTargetPosition((int)(targetPos+.5));
        robot.carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.carousel.setPower(.5);
        if (!robot.carousel.isBusy()){
            curPos = targetPos;
            finished = true;
        }
    }

}
