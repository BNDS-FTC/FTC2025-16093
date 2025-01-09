package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;

public class GrabAction extends Action {
    private int toleranceRange = 150;
    private SuperStructure upper;
    //Params not in super class
    private double pos;
    private long timeOnStart;

    public GrabAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
        timeOnStart = System.currentTimeMillis();
    }

    public GrabAction(SuperStructure upper, double pos, int waitTime){
        this.upper = upper;
        this.pos = pos;
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

    public String toString() {
        return getType() + " Pos " + this.pos;
    }

    public String getType(){
        return "GrabAction";
    }

    public void actuate() {
        upper.setGrabPos(pos);
    }

}
