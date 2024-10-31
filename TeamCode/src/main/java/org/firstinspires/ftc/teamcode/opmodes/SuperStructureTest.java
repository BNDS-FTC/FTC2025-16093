package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp(name = "SuperStructure Test")
public class SuperStructureTest extends LinearOpMode {
    private SuperStructure upper;
    @Override
    public void runOpMode() throws InterruptedException{
        upper = new SuperStructure(
                this,
                () -> {
                    logic_period();
                    drive_period();
                });
        XCYBoolean intakeFar =new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean intakeNear = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean resetPos = new XCYBoolean(()->gamepad1.x);
        XCYBoolean releaseHigh = new XCYBoolean(()->gamepad1.y);
        XCYBoolean intakeIn = new XCYBoolean(()->gamepad1.right_bumper);
        XCYBoolean intakeOut = new XCYBoolean(()->gamepad1.left_bumper);
        XCYBoolean grabOpen = new XCYBoolean(()->gamepad1.a);
        XCYBoolean grabClose = new XCYBoolean(()->gamepad1.b);
        XCYBoolean wristIntake = new XCYBoolean(()->gamepad1.dpad_left);
        XCYBoolean wristDrop = new XCYBoolean(()->gamepad1.dpad_right);

        upper.resetSlide();

        waitForStart();

        while(opModeIsActive()){

            upper.update();
            telemetry.addData("arm: ",upper.getArmPosition());
            telemetry.addData("slideL: ",upper.getSlideLeftPosition());
            telemetry.addData("slideR: ",upper.getSlideRightPosition());
            telemetry.update();
            XCYBoolean.bulkRead();
        }
    }

    private void drive_period() {
        //there's nothing here
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }
}