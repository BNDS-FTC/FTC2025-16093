package org.firstinspires.ftc.teamcode.actions.actioncore;

public class WaitAction extends Action {
    private int waitTime = 150;
    //Params not in super class
    private long timeOnStart;

    public WaitAction(int waitTime){
        this.waitTime = waitTime;
        timeOnStart = System.currentTimeMillis();
    }


    public boolean canStartNext(){
        if(System.currentTimeMillis() - timeOnStart > waitTime){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public boolean isFinished() {
        return canStartNext();
    }


    public String toString() {
        return returnType() + " Time " + this.waitTime;
    }

    public String returnType(){
        return "WaitAction";
    }
}
