package org.firstinspires.ftc.teamcode.uppersystems;

public abstract class Action {
    private int error;
    private boolean finishCondition;
    private SuperStructure upper;


    public void setToleranceRange(boolean finishCondition){
        this.finishCondition = finishCondition;
    }

    public int getError(){
        return 0;
    }

    public void actuate(){

    }

    public boolean isFinished(){
        return false;
    }



}
