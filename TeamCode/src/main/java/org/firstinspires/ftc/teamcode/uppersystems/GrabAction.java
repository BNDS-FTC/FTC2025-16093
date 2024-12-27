package org.firstinspires.ftc.teamcode.uppersystems;

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

    public boolean isFinished(){
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
        return "GrabAction";
    }

    public void actuate() {
        upper.setGrabPos(pos);
    }

}
