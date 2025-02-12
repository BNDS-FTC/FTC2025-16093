package org.firstinspires.ftc.teamcode.references;

import java.util.function.BooleanSupplier;

public class TimerBoolean extends XCYBoolean {

    private boolean current_val = false, last_val;
    private int requiredMs;
    private long startTime = -1;

    public TimerBoolean(BooleanSupplier condition, int requiredMs) {
        super(condition);
        this.requiredMs = requiredMs;
    }

    public void read() {
        if(trueCondition != null){
            last_val = current_val;
            current_val = trueCondition.getAsBoolean();
            if(toTrue()){
                startTime = System.currentTimeMillis();
            }
            if(toFalse()){
                startTime = -1;
            }
        }
    }

    public boolean trueTimeReached(){
        if(getLoopsSinceTrue() < requiredMs){
            return false;
        }else{
            return true;
        }
    }

    public long getLoopsSinceTrue(){
        return System.currentTimeMillis() - startTime;
    }

}