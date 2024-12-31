package org.firstinspires.ftc.teamcode.references;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class XCYBoolean {
    private static final ArrayList<XCYBoolean> allInstance = new ArrayList<>();

    private final BooleanSupplier trueCondition;
    protected boolean current_val = false, last_val;

    public XCYBoolean(BooleanSupplier condition) {
        trueCondition = condition;
        initialRead();
        allInstance.add(this);
    }

    public void read() {
        last_val = current_val;
        current_val = trueCondition.getAsBoolean();
    }

    public void initialRead() {
        last_val = current_val;
        current_val = trueCondition.getAsBoolean();
    }

    public boolean get() {
        return current_val;
    }

    public boolean toTrue() {
        return !last_val && current_val;
    }

    public boolean toFalse() {
        return last_val && !current_val;
    }

    public boolean isChanged(){
        return toFalse()||toTrue();
    }

    public static void bulkRead() {
        for (XCYBoolean b : allInstance) {
            b.read();
        }
    }

    public void deactivate(){
        allInstance.remove(this);
    }
}