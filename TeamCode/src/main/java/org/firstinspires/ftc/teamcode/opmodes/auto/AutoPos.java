package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.SuperStructure;

@Autonomous(name="2AutoPos")
public class AutoPos extends LinearOpMode {

    SuperStructure upper;

    @Override
    public void runOpMode() throws InterruptedException {

        upper = new SuperStructure(
                this,
                () -> {upper.update();}, 0);

        waitForStart();

        while(opModeIsActive()){
            upper.setTailPos(SSValues.TAIL_AUTO_POS);
            upper.setAscentState(SuperStructure.AscentState.ASCENT_DOWN);
            upper.setArmByP(SSValues.AUTO_ARM_OFFSET,1);
        }
    }
}
