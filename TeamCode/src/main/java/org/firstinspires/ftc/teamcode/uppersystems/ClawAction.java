package org.firstinspires.ftc.teamcode.uppersystems;

public class ClawAction extends Action {
    private int toleranceRange = 150;
    private SuperStructure upper;
    //Params not in super class
    private double pos;
    private long timeOnStart;

    public ClawAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
        timeOnStart = System.currentTimeMillis();
    }

    public ClawAction(SuperStructure upper, double pos, int toleranceRange){
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
        upper.setClawLeftPos(pos);
        upper.setClawRightPos(pos);
    }

    //Functions not in super class
    public void setPos(double pos) {
        pos = this.pos;
    }

}
