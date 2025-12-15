package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import java.util.Locale;

public class PrepareLaunchColor extends CarouselOperations {

    int index;
    boolean posSet;

    public PrepareLaunchColor(int index) {
        this.index = index;
    }

    @NonNull
    @Override
    public String toString() {
        return (String.format(Locale.US, "%s(%d):%b", getClass().getSimpleName(), index, posSet));
    }

    @Override
    public void loop() {
        super.loop();
        int bay = 1;
        IterativeScanObelisk.OBELISK_PATTERN pat = IterativeScanObelisk.curPattern;
        if (!posSet) {
            posSet = true;
            if (index == 1){
                if (pat == IterativeScanObelisk.OBELISK_PATTERN.PPG){
                    bay = 1;
                }
                else if (pat == IterativeScanObelisk.OBELISK_PATTERN.PGP){
                    bay = 2;
                }
                else if (pat == IterativeScanObelisk.OBELISK_PATTERN.GPP){
                    bay = 3;
                }
            }
            else if (index == 2){
                if (pat == IterativeScanObelisk.OBELISK_PATTERN.PPG){
                    bay = 2;
                }
                else if (pat == IterativeScanObelisk.OBELISK_PATTERN.PGP){
                    bay = 3;
                }
                else if (pat == IterativeScanObelisk.OBELISK_PATTERN.GPP){
                    bay = 1;
                }
            }
            else if (index == 3){
                if (pat == IterativeScanObelisk.OBELISK_PATTERN.PPG){
                    bay = 3;
                }
                else if (pat == IterativeScanObelisk.OBELISK_PATTERN.PGP){
                    bay = 1;
                }
                else if (pat == IterativeScanObelisk.OBELISK_PATTERN.GPP){
                    bay = 2;
                }
            }
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
