package org.firstinspires.ftc.teamcode.uppersystems;

import org.firstinspires.ftc.teamcode.references.SSValues;

public class ClawAction extends Action {
    private int toleranceRange = 50;
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

    public ClawAction(SuperStructure upper, double posL, double posR, int waitTime){
        this.upper = upper;
        this.posL = posL;
        this.posR = posR;
        this.toleranceRange = waitTime;
        timeOnStart = System.currentTimeMillis();
    }

    public int getError() {
        return 0;
    }


    public boolean canStartNext(){
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

    public String returnType(){
        return "ClawAction";
    }

    public String toString() {
        return returnType() + " PosL " + this.posL + " posR" + this.posR;
    }

}
