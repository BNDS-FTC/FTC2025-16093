package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;

public class ServoAction extends Action {
    private int toleranceRange = 80;
    protected SuperStructure upper;
    //Params not in super class
    protected double pos;
    protected long timeOnStart;

    public ServoAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
        timeOnStart = System.currentTimeMillis();
    }

    public ServoAction(SuperStructure upper, double pos, int waitTime){
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
        return returnType() + " Pos " + this.pos;
    }

    public String returnType(){
        return "ServoAction";
    }


}
