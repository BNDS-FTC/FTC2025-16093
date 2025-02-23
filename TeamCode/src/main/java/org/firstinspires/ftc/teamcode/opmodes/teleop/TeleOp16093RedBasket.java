package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.actioncore.Action;

//@Photon
@TeleOp(name = "16093 Red Basket Double TeleOp")
@Disabled
public class TeleOp16093RedBasket extends TeleOpMaster {
    @Override
    public void runOpMode() throws InterruptedException {

        initTeleOp(()->(upper.alphaAdjustedSampleColor() == 1||upper.alphaAdjustedSampleColor() == 0), -90);

        // Wait until play button is pressed

        waitForStart();


        // Main control loop while op mode is active
        while (opModeIsActive() && !isStopRequested()) {
            update.run();
            Action.buildSequence(update);

        }
    }

}