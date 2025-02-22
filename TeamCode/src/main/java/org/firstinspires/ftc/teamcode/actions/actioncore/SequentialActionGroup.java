package org.firstinspires.ftc.teamcode.actions.actioncore;

import org.firstinspires.ftc.teamcode.SuperStructure;

/**
 * This is essentially a mini buildSequence so that you can put Actions in sequence in
 * places where only one Action is supposed to be
 */

public class SequentialActionGroup extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private Action[] actions;
    private Action currentAction;
    private Runnable update;

    public SequentialActionGroup(Runnable update, Action...actions){
        this.update = update;
        this.actions = actions;
    }

    public int getError() {
        int sum = 0;
        for(Action a:actions){
            sum += a.getError();
        }
        return sum/actions.length;
    }

    public boolean canStartNext(){
        boolean canStart = true;
        for(Action a:actions){
            canStart = a.canStartNext() && canStart;
        }
        return canStart;
    }

    public boolean isFinished(){
        boolean canFinish = true;
        for(Action a:actions){
            canFinish = a.isFinished() && canFinish;
        }
        return canFinish;
    }

    public void actuate() {
        for (int i=0;i < actions.length;i++) {
            currentAction = actions[i];
            currentAction.actuate();

            while(!currentAction.canStartNext()){
                update.run();

                if(currentAction.isFinished()){
                    currentAction.stop();
                }
            }
        }
    }

    public void stop(){
        for(Action a:actions){
            a.stop();
        }
    }

    public void forceStop(){
        for(Action a:actions){
            a.forceStop();
        }
    }

    public String toString(){
        return returnType()+" "+ currentAction.returnType();
    }

    public String returnType(){
        return "SequentialAction";
    }

}
