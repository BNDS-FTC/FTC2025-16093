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
        this.readCondition = readCondition;
    }

    public void read() {
        if(readCondition.getAsBoolean()){
            if(trueCondition != null){
                last_val = current_val;
                current_val = trueCondition.getAsBoolean();
                if(current_val && !last_val){
                    startTime = System.currentTimeMillis();
                }
                if(!current_val && last_val){
                    startTime = Integer.MAX_VALUE;
                }
            }
        }
        else{
            last_val = current_val;
            current_val = false;
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