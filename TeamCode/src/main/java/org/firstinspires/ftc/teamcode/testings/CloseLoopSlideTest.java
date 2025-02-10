package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;

@TeleOp(name = "CloseLoop Slide Test")
public class CloseLoopSlideTest extends LinearOpMode {
    private DcMotorEx mSlideRight = null;
    private DcMotorEx mSlideLeft = null;
    private SuperStructure upper;
//    public NewMecanumDrive drive;
    Runnable update;


    @Override
    public void runOpMode() throws InterruptedException {
        upper = new SuperStructure(
                this,
                () -> {
                }, 0);

        upper.resetSlide();

        update = () -> {
            logic_period();
            drive_period();
            upper.update();
//            gamepad_inputs();
//            if (forceStop.toTrue()) {
//                Action.stopBuilding = true;
//            }
//            if (forceStop.toFalse()) {
//                Action.stopBuilding = false;
//            }
        };

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 10));
            }
            if(gamepad1.b){
                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 10));
            }
            Action.buildSequence(update);


            upper.update();
            telemetry.addData("slideL: ", upper.getSlideLeftPosition());
            telemetry.addData("slideR: ", upper.getSlideRightPosition());
            telemetry.addData("Power left", mSlideLeft.getPower());
            telemetry.addData("Power left", mSlideRight.getPower());
            telemetry.addData("right_velocity", mSlideRight.getVelocity());
            telemetry.addData("left_velocity", mSlideLeft.getVelocity());
            XCYBoolean.bulkRead();

        }
    }

    private void drive_period(){

    }
    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }
}