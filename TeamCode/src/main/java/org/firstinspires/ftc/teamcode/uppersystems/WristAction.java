package org.firstinspires.ftc.teamcode.uppersystems;

public class WristAction extends Action {
    private int toleranceRange = 300;
    private SuperStructure upper;
    //Params not in super class
    private double pos;
    private long timeOnStart;

    public WristAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
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

    //Functions not in super class
    public void setPos(double pos) {
        pos = this.pos;
    }

}
