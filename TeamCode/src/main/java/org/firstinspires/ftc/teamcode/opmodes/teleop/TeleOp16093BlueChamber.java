package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.references.SSValues;

//@Photon
@TeleOp(name = "16093 Blue Chamber Double TeleOp")
public class TeleOp16093BlueChamber extends TeleOp16093 {
    @Override
    public void runOpMode() throws InterruptedException {

        initTeleOp(()->(upper.alphaAdjustedSampleColor().equals("blue"))&&
                (upper.getWristPosition() == SSValues.WRIST_INTAKE || upper.getWristPosition() == SSValues.WRIST_INTAKE_SPECIMEN));

        // Wait until play button is pressed

        waitForStart();


        // Set intake to default stop position and initialize operation mode
//        upper.startIntake();

        // Main control loop while op mode is active
        while (opModeIsActive() && !isStopRequested()) {
            update.run();
            Action.buildSequence(update);

        }
    }

}