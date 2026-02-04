package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

import java.util.LinkedList;

import java.util.Locale;

public class NestedQOp extends RobotOperation {
    private final LinkedList<IterativeRobotParent.OperationData> pendingOperations = new LinkedList<IterativeRobotParent.OperationData>();
    private final LinkedList<IterativeRobotParent.OperationData> completeOperations = new LinkedList<IterativeRobotParent.OperationData>();

    IterativeRobotParent.OperationData activeOperation = null;
    ElapsedTime operationsRunTime = new ElapsedTime();
    private boolean finished;

    public void addOperation(RobotOperation newOp) {
        pendingOperations.add(new IterativeRobotParent.OperationData(newOp));
    }

    public void loop() {
        if (finished) return;
        if (activeOperation == null) {
            if (!pendingOperations.isEmpty()) {
                activeOperation = pendingOperations.removeFirst();
                activeOperation.startTime = operationsRunTime.seconds();
                activeOperation.op.init(robot);
            } else {
                finished = true;
                return;
            }
        }

        // If there is an active operation, call its loop
        if (activeOperation != null) {
            activeOperation.loopCount++;
            activeOperation.op.loop();

            // If the current operation is finished stop it and clear the active operation field
            if (activeOperation.op.isFinished()) {
                // For debugging we track all operations that have been run to completion, we can
                // dump them out using displayOperations or review them in the debugger
                activeOperation.endTime = operationsRunTime.seconds();
                completeOperations.add(activeOperation);
                activeOperation.op.stop();
                activeOperation = null;
            }
        }
    }

    @NonNull
    public String toString() {
        if (activeOperation == null) {
            return "opQueue: null";
        }
        return String.format(Locale.US, "opQueue: %s", activeOperation);
    }

    public String opsToString() {
        StringBuilder ops = new StringBuilder();
        ops.append(String.format(Locale.US, "%s:%b \n", getClass().getSimpleName(), finished));
        for (IterativeRobotParent.OperationData op : completeOperations) {
            ops.append(String.format("   - %s\n", op.toString()));
        }
        return ops.toString();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
