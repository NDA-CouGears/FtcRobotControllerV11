package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PrepareLoad extends CarouselOperations{
    public PrepareLoad(int bay){
        // Bill: This needs to move into loop after we update curPos, offset and zero since they
        // change throughout the run and may be different by the time this operation is run
        // We may also want to move the math into a constant instead of redoing it each time
        if (bay == 1){
            targetPos = zero + CTR/2;
        }
        else if (bay == 2){
            targetPos = zero + (2*CTR/3);
        }
        else if (bay == 3){
            targetPos = zero + (CTR/3);
        }
    }

    @Override
    public void loop() {
        // Bill: We need to move the update of offset and zero to after we update curPos since they
        // depend on it. We could move the assignment of curPos to the parent loop or call
        // super.loop() below the assignment here, but I think it make more sense int he parent
        super.loop();
        robot.carousel.setTargetPosition((int)(targetPos+.5));
        robot.carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.carousel.setPower(.5);
        curPos = robot.carousel.getCurrentPosition();
        if (!robot.carousel.isBusy()){
            curPos = targetPos;
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
