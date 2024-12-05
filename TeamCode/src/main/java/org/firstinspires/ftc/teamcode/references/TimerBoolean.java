package org.firstinspires.ftc.teamcode.references;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class TimerBoolean extends XCYBoolean {

    private final BooleanSupplier trueCondition;
    private boolean current_val = false, last_val;
    private long timeOnToTrue = 0;

    public TimerBoolean(BooleanSupplier condition) {
        super(condition);
        trueCondition = condition;
    }

    public void read() {
        if(trueCondition != null){
            last_val = current_val;
            current_val = trueCondition.getAsBoolean();
            if(toTrue()){
                timeOnToTrue = System.currentTimeMillis();
            }
        }
    }

    public boolean trueTimeReached(int requiredMs){
        int timeSinceToTrue = (int)getTimeSinceToTrue();
        if(timeSinceToTrue < requiredMs){
            return false;
        }else{
            return true;
        }
    }

    public long getTimeSinceToTrue(){
        return System.currentTimeMillis() - timeOnToTrue;
    }

}