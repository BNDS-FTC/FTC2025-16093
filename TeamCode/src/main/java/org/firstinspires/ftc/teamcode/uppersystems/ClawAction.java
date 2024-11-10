package org.firstinspires.ftc.teamcode.uppersystems;

import org.firstinspires.ftc.teamcode.references.SSValues;

public class ClawAction extends Action {
    private int toleranceRange = 150;
    private SuperStructure upper;
    //Params not in super class
    private double posL;
    private double posR;
    private long timeOnStart;

    public ClawAction(SuperStructure upper, double posL, double posR){
        this.upper = upper;
        this.posL = posL;
        this.posR = posR;
        timeOnStart = System.currentTimeMillis();
    }

    public ClawAction(SuperStructure upper, double posL, double posR, int toleranceRange){
        this.upper = upper;
        this.posL = posL;
        this.posR = posR;
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

    public void actuate() {
        upper.setClawLeftPos(posL);
        upper.setClawRightPos(posR);
    }

}
