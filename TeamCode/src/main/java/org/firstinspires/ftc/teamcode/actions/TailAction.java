package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.references.SSValues;

public class TailAction extends ServoAction{

    public TailAction(SuperStructure upper, double pos){
        super(upper, pos);
    }

    public TailAction(SuperStructure upper, double pos, int waitTime){
        super(upper, pos, waitTime);
    }

    public void actuate() {
        upper.setTailPos(pos);
    }

}
