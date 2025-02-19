package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.references.SSValues;

@TeleOp
@Disabled
public class CBringArmDownToAutoPos extends LinearOpMode {

    SuperStructure upper;

    @Override
    public void runOpMode() throws InterruptedException {

        upper = new SuperStructure(
                this,
                () -> {upper.update();}, 0);

        waitForStart();

        while(opModeIsActive()){
            upper.setArmByP((SSValues.AUTO_ARM_OFFSET-SSValues.ARM_UP),1);
        }
    }
}
