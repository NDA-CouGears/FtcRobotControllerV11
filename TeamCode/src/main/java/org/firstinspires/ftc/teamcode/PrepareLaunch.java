package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PrepareLaunch extends CarouselOperations{
    /* private float launchB1 = zero + 0;
    private float launchB2 = zero + (CTR/6);
    private float launchB3 = zero + (2*CTR/3); */

    public PrepareLaunch(int bay){
        if (bay == 1){
            targetPos = zero;
        }
        else if (bay == 2){
            targetPos = zero + (CTR/6);
        }
        else if (bay == 3){
            targetPos = zero + (2*CTR/3);
        }
    }

    @Override
    public void loop() {
        super.loop();
        robot.carousel.setTargetPosition((int)(targetPos+.5));
        robot.carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.carousel.setPower(.5);
        curPos = robot.carousel.getCurrentPosition();
        if (robot.carousel.getCurrent(CurrentUnit.AMPS) == 0){
            curPos = targetPos;
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
