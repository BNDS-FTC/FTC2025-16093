package org.firstinspires.ftc.teamcode.uppersystems;

public class WristAction extends Action {
    private int toleranceRange = 80;
    private SuperStructure upper;
    //Params not in super class
    private double pos;
    private long timeOnStart;

    public WristAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
        timeOnStart = System.currentTimeMillis();
    }

    public WristAction(SuperStructure upper, double pos, int waitTime){
        this.upper = upper;
        this.pos = pos;
        this.toleranceRange = waitTime;
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
        upper.setWristPos(pos);
    }

}
