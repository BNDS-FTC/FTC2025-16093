package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.references.SSValues;

//@Photon
@TeleOp(name = "16093 Red Basket Double TeleOp")
public class TeleOp16093RedBasket extends TeleOp16093 {
    @Override
    public void runOpMode() throws InterruptedException {

        initTeleOp(()->(upper.alphaAdjustedSampleColor().equals("yellow")||upper.alphaAdjustedSampleColor().equals("red")));

        // Wait until play button is pressed

        waitForStart();


        // Main control loop while op mode is active
        while (opModeIsActive() && !isStopRequested()) {
            update.run();
            Action.buildSequence(update);

        }
    }

}