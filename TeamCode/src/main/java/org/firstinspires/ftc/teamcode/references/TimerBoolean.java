package org.firstinspires.ftc.teamcode.references;

import java.util.function.BooleanSupplier;

public class TimerBoolean extends XCYBoolean {

    private final BooleanSupplier trueCondition;
    private boolean current_val = false, last_val;
    private int loopsSinceTrue = 0;

    public TimerBoolean(BooleanSupplier condition) {
        super(condition);
        trueCondition = condition;
    }

    public void read() {
        if(trueCondition != null){
            last_val = current_val;
            current_val = trueCondition.getAsBoolean();
            if(toTrue()){
                loopsSinceTrue = 0;
            }else{
                loopsSinceTrue++;
            }
        }
    }

    public boolean trueTimeReached(int requiredMs){
        if(loopsSinceTrue < requiredMs){
            return false;
        }else{
            return true;
        }
    }

    public int getLoopsSinceTrue(){
        return loopsSinceTrue;
    }

}