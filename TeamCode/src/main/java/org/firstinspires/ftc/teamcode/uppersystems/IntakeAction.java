package org.firstinspires.ftc.teamcode.uppersystems;

import org.firstinspires.ftc.teamcode.references.SSValues;

public class IntakeAction extends Action{

    private int toleranceRange = 150;
    private SuperStructure upper;
    //Params not in super class
    private double pos;
    private long timeOnStart;

    public IntakeAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
        timeOnStart = System.currentTimeMillis();
    }

    public IntakeAction(SuperStructure upper, double pos, int toleranceRange){
        this.upper = upper;
        this.pos = pos;
        this.toleranceRange = toleranceRange;
        timeOnStart = System.currentTimeMillis();
    }

    public int getError() {
        return 0;
    }


    public boolean isFinished(){
        if(System.currentTimeMillis() - timeOnStart > toleranceRange){
            return true;
        }else{
            return false;
        }
    }

    public boolean canStartNext(){
        if(System.currentTimeMillis() - timeOnStart > toleranceRange){
            return true;
        }else{
            return false;
        }
    }

    public void forceStop(){
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        toleranceRange = -1;
    }

    public void actuate() {
        upper.setIntake(pos);
    }

}
