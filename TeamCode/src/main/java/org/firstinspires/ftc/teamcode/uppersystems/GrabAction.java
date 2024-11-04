package org.firstinspires.ftc.teamcode.uppersystems;

public class GrabAction extends Action {
    private int toleranceRange = 300;
    private SuperStructure upper;
    //Params not in super class
    private double pos;
    private long timeOnStart;

    public GrabAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
        timeOnStart = System.currentTimeMillis();
    }

    public GrabAction(SuperStructure upper, double pos, int toleranceRange){
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

    public void actuate() {
        upper.setGrabPos(pos);
    }

    //Functions not in super class
    public void setPos(double pos) {
        pos = this.pos;
    }

}
