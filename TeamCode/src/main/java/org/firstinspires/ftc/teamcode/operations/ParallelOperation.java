package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;

/**
 * An operation that can run multiple other operations at the same time. It can be configured to
 * finish when any of the parallel operations finish or when all finish.
 */
public class ParallelOperation extends RobotOperation {
    boolean waitForAll;
    private final ArrayList<RobotOperation> operations = new ArrayList<>();

    /**
     * Create a single operation with a list of operations to run in parallel. A call to any method
     * on this operation in turn calls the same method on all children. It is worth noting this is
     * not simultaneous, each child method is called in the order they were added, so if there is
     * possible interaction between child operations keep this in mind.
     *
     * @param waitForAll If true this operation is not complete until all child operations are, if
     *                   false this is complete when any single child completes
     * @param ops A coma separated list of child operations to run in parallel
     */
    public ParallelOperation(boolean waitForAll, RobotOperation... ops) {
        this.waitForAll = waitForAll;
        operations.addAll(Arrays.asList(ops));
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder childStrings = new StringBuilder();
        for (RobotOperation op:operations) {
            childStrings.append("[");
            childStrings.append(op);
            childStrings.append("]");
        }
        childStrings.append(waitForAll);
        return(childStrings.toString());
    }

    @Override
    public void init(IterativeRobotParent robot) {
        super.init(robot);
        operations.forEach(op -> {
            op.init(this.robot);
        });
    }

    @Override
    public void loop() {
        operations.forEach(RobotOperation::loop);
    }

    @Override
    public boolean isFinished() {
        if (waitForAll) {
            for (RobotOperation op : operations) {
                // if any are not finished we have to keep waiting
                if (!op.isFinished()) {
                    return false;
                }
            }

            // We made it out of the loop so all of the operations must be finished
            return true;
        }
        else {
            for (RobotOperation op : operations) {
                // Not waiting for all, so if one is finished this parallel op is finsihed
                if (op.isFinished()) {
                    return true;
                }
            }

            // None of the ops are finished so the parallel op is not
            return false;
        }
    }

    @Override
    public void stop() {
        operations.forEach(RobotOperation::stop);
    }
}
