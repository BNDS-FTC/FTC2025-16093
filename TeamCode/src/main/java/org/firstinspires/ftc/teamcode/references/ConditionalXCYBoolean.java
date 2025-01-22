package org.firstinspires.ftc.teamcode.references;

import java.util.function.BooleanSupplier;

public class ConditionalXCYBoolean extends XCYBoolean{
    private BooleanSupplier readCondition;
    public ConditionalXCYBoolean(BooleanSupplier condition, BooleanSupplier readCondition){
        super(condition);
        this.readCondition = readCondition;
    }

    @Override
    public void read() {
        if(readCondition.getAsBoolean()){
            super.read();
        }
        else{
            last_val = current_val;
            current_val = false;
        }
    }
}
