package org.firstinspires.ftc.teamcode.references;

import java.util.function.BooleanSupplier;

public class TimerBoolean extends ConditionalXCYBoolean {

    private boolean current_val = false, last_val;
    private int requiredMs;
    private long startTime = 1000000;
    private BooleanSupplier readCondition;


    public TimerBoolean(BooleanSupplier condition, BooleanSupplier readCondition, int requiredMs) {
        super(condition, readCondition);
        this.requiredMs = requiredMs;
    }

    public void conditionalRead() {
        if(trueCondition != null){
            last_val = current_val;
            current_val = trueCondition.getAsBoolean();
            if(current_val && !last_val){
                startTime = System.currentTimeMillis();
            }
            if(!current_val){
                startTime = System.currentTimeMillis();
            }
        }

    }

    public boolean trueTimeReached(){
        if(getTimeSinceTrue() < requiredMs){
            return false;
        }else{
            return true;
        }
    }

    public long getTimeSinceTrue(){
        return System.currentTimeMillis() - startTime;
    }

}