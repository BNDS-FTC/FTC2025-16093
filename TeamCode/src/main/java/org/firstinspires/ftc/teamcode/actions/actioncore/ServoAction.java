package org.firstinspires.ftc.teamcode.actions.actioncore;

import org.firstinspires.ftc.teamcode.SuperStructure;

/**
 * Servo Actions - i.e. actions where the finish condition is determined by a timer rather than an encoder
 */

public class ServoAction extends Action {
    private int waitTime = 80;
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
        this.waitTime = waitTime;
        timeOnStart = System.currentTimeMillis();
    }


    public int getError() {
        return 0;
    }

    public boolean canStartNext(){
        if(System.currentTimeMillis() - timeOnStart > waitTime){
            return true;
        }else{
            return false;
        }
    }
    public boolean isFinished(){
        return canStartNext();
    }

    public String toString() {
        return returnType() + " Pos " + this.pos;
    }

    public String returnType(){
        return "ServoAction";
    }


}
