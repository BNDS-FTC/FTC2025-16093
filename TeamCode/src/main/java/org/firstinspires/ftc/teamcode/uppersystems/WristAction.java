package org.firstinspires.ftc.teamcode.uppersystems;

public class WristAction extends Action {
    private int toleranceRange = 300;
    private SuperStructure upper;
    //Params not in super class
    private double pos;

    public WristAction(SuperStructure upper, double pos){
        this.upper = upper;
        this.pos = pos;
    }

    public int getError() {
        return 0;
    }

    public void setToleranceRange(int error) {
        this.toleranceRange = error;
    }

    public boolean isFinished(){
        return true;
    }

    public void actuate() {
        upper.setWristPos(pos);
    }

    //Functions not in super class
    public void setPos(double pos) {
        pos = this.pos;
    }

}
