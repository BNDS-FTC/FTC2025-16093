package org.firstinspires.ftc.teamcode.uppersystems;

import java.lang.reflect.Array;
import java.util.ArrayList;

import XCYOS.TaskChainBuilder;

public class Action {
    private int error;
    private SuperStructure upper;
    private long timeOnStart;
    public final static ArrayList<Action> actions = new ArrayList<>(6);
    public static boolean stopBuilding = false;


    public int getError(){
        return 0;
    }

    public void actuate(){

    }

    public boolean isFinished(){
        return false;
    }

    public boolean canStartNext(){
        return false;
    }


    public String returnType() {
        return "Action";
    }

    public void forceStop(){
    }

    public void stop(){
        actions.remove(this);
    }

    public String toString(){
        return "WHY THE HELL IS THIS BEING CALLED ANYWAY ANNIE ZHUANG???";
    }

    static Action currentAction;
    public static void buildSequence(Runnable runWhileBuilding){
        if(!actions.isEmpty()){
            for (int i=0;i < actions.size();i++) {
                currentAction = actions.get(i);
                currentAction.actuate(); // Execute current action

                while(!currentAction.canStartNext()){
                    runWhileBuilding.run();

                    if(stopBuilding){
                        currentAction.forceStop();
                        actions.clear();
                        stopBuilding = false;
                        break;
                    }

                    if(currentAction.isFinished()){ //|| System.currentTimeMillis() - currentAction.timeOnStart > 10000
                        currentAction.stop();
                    }
                }
            }
            actions.clear(); // Clear completed actions and reset mode
        }
    }

    public static String showCurrentAction(){
        if(currentAction!=null){
            return currentAction.toString();
        }else{
            return "NO ACTION YET!";
        }
    }
    public static void clearActions(){
        currentAction = null;
        Action.actions.clear();
    }

}
