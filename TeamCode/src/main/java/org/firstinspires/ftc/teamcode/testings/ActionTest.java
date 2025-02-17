package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.Action;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;

@TeleOp(name="Action Test")
@Config
public class ActionTest extends LinearOpMode {

    static int pos = 600;
    Runnable update;
    SuperStructure upper;
    double oldTime = 0;
    int count = 0;

    @Override
    public void runOpMode() {
        XCYBoolean a = new XCYBoolean(()->gamepad1.a);
        XCYBoolean b = new XCYBoolean(()->gamepad1.b);
        upper = new SuperStructure(this, ()->{},0);
        upper.resetArmEncoder();
        Action.setOpModeActive(()->opModeIsActive());

        update = () -> {
            logic_period();
            upper.update();
        };

        upper.setUpdateRunnable(update);

        waitForStart();

        while (opModeIsActive()) {
            if(Action.actions.isEmpty()){
                if(a.toTrue()){
                    Action.actions.add(new SlideAction(upper, pos));
                }
                if(b.toTrue()){
                    Action.actions.add(new SlideAction(upper, 0));
                }
            }

            Action.buildSequence(update);
            update.run();

        }
    }

    private void logic_period() {
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        XCYBoolean.bulkRead();
        count ++;
        telemetry.addData("Loops since start: ", count);
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

        telemetry.addData("Arm Position: ", upper.getArmPosition());
        telemetry.addData("Slide Position: ", upper.getSlidesPosition());
        telemetry.addLine("");
        telemetry.addData("Arm Power", upper.getArmPower());
        telemetry.addData("Slide Power:", upper.getSlidePower());
        telemetry.addLine("");

//        telemetry.addData("Arm Target Position", upper.getArmTargetPosition());
//        telemetry.addData("Slide Target Position", upper.getSlideTargetPosition());
        telemetry.addData("Current Sequence", upper.getSequence());
        telemetry.addData("Previous Sequence", upper.getPreviousSequence());
        telemetry.addLine("");
//        telemetry.addData("Drive Mode", driveMode);
        telemetry.addData("Action Stop?", Action.stopBuilding);
//        telemetry.addData("Touch Sensor Pressed?", upper.mTouchSensor.isPressed());
//        telemetry.addData("Slide Lock Position", upper.getSlideLockPosition());
//        telemetry.addData("Color Sensor values",upper.getColorRGBAValues(15));
//        telemetry.addData("AutoGrab: ", autoGrabSample.get());
//        telemetry.addData("AutoGrab toTrue: ", autoGrabSample.toTrue());
        telemetry.addLine(Action.showCurrentAction());
        telemetry.update();

//        telemetry_M.addData("Slide Power:", upper.getSlidePower());
//        telemetry_M.addData("Arm Power", upper.getArmPower());
//        telemetry_M.update();
//        for (LynxModule module : allHubs) {
//            module.clearBulkCache();
//        }
    }
}
