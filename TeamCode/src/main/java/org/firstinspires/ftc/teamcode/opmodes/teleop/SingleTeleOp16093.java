package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.TimerBoolean;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;

//@Photon
@TeleOp(name = "16093 Single TeleOp")
public class SingleTeleOp16093 extends TeleOp16093 {
    @Override
    public void runOpMode() throws InterruptedException {

        initTeleOp(()->upper.colorSensorCovered());

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

    @Override
    public void key_binds() {
//        if(opModeIsActive()){
        resetPos = new XCYBoolean(() -> gamepad1.left_stick_button && !gamepad1.right_stick_button);
        resetOdo = new XCYBoolean(() -> gamepad1.right_stick_button && !gamepad1.left_stick_button);
        switchDrive = new XCYBoolean(() -> gamepad1.right_stick_button && gamepad1.left_stick_button);
        changeGrab = new XCYBoolean(() -> gamepad1.right_trigger > 0.1);
        slideLonger = new XCYBoolean(() -> gamepad1.dpad_up && !(gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right));
        slideShorter = new XCYBoolean(() -> gamepad1.dpad_down);
        forceStop = new XCYBoolean(() -> gamepad1.b);
        releaseHigh = new XCYBoolean(() -> gamepad1.y);
        releaseLow = new XCYBoolean(() -> gamepad1.a);
        highChamberPlace = new XCYBoolean(() -> gamepad1.right_bumper && (upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER_AIM || (upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER)));
        highChamberAim = new XCYBoolean(() -> gamepad1.left_bumper && upper.getSequence() == SuperStructure.Sequences.RUN);
        wristHeightSwitch = new XCYBoolean(() -> gamepad1.right_stick_button);
        altWristHeightSwitch = new XCYBoolean(() -> gamepad1.left_trigger > 0);
        armDownByPower = new XCYBoolean(() -> gamepad1.options && !(gamepad1.back));
        manualResetEncoders = new XCYBoolean(() -> gamepad1.back && gamepad1.options);
        goToLastStoredPos = new XCYBoolean(() -> gamepad1.dpad_left && !(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right));
        storeThisPos = new XCYBoolean(() -> gamepad1.dpad_right && !(gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up));
        ascentDown = new XCYBoolean(() -> gamepad1.x);
        getWallSpecimen = new XCYBoolean(() -> gamepad1.left_trigger > 0 && upper.getSequence() == SuperStructure.Sequences.RUN);

        resetArm = new TimerBoolean(() -> upper.getTouchSensorPressed(), ()->upper.getSequence() == SuperStructure.Sequences.RUN, 200);

        openLoopSlideController = () -> gamepad1.right_stick_y;

//        }
    }
}