package org.firstinspires.ftc.teamcode.uppersystems;

public class WaitAction extends Action {
    private int toleranceRange = 150;
    //Params not in super class
    private long timeOnStart;

    public WaitAction(int waitTime){
        this.toleranceRange = waitTime;
        timeOnStart = System.currentTimeMillis();
    }


    public boolean canStartNext(){
        if(System.currentTimeMillis() - timeOnStart > toleranceRange){
            return true;
        }else{
            return false;
        }
    }

}
