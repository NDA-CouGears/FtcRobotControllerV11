package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PrepareLoad extends CarouselOperations{
    int bay;
    public PrepareLoad(int bay){
        this.bay = bay;
    }

    @Override
    public void loop() {
        super.loop();
        if (bay == 1){
            targetPos = zero + CTR/2;
        }
        else if (bay == 2){
            targetPos = zero + (2*CTR/3);
        }
        else if (bay == 3) {
            targetPos = zero + (CTR / 3);
        }
        robot.carousel.setTargetPosition((int)(targetPos+.5));
        robot.carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.carousel.setPower(.5);
        curPos = robot.carousel.getCurrentPosition();
        if (!robot.carousel.isBusy()){
            curPos = targetPos;
            finished = true;
        }
    }

}
