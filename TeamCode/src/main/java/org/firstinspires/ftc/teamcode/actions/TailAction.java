package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.references.SSValues;

public class TailAction extends Action{

    private int toleranceRange = 150;
    private SuperStructure upper;
    //Params not in super class
    private double pos;
    private long timeOnStart;

    public TailAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
        timeOnStart = System.currentTimeMillis();
    }

    public TailAction(SuperStructure upper, double pos, int toleranceRange){
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


    public void actuate() {
        upper.setTailPos(pos);
    }

}
