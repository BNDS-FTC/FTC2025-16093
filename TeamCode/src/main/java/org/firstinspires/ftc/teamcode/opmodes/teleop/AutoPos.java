package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.SuperStructure;

@TeleOp(name="2AutoPos")
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
