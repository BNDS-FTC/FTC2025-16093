package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp(name = "TeleOp 16093")
public class TeleOp16093 extends LinearOpMode {
    public SuperStructure upper;
    public NewMecanumDrive drive;
    private Sequences sequence;
    private Pose2d current_pos;

    @Override
    public void runOpMode() throws InterruptedException{
        SuperStructure upper = new SuperStructure(
                this,
                () -> {
                    logic_period();
                    drive_period();
                });

        ///////////////////////////GAMEPAD1//////////////////////////////////////////////////////
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

        ///////////////////////////GAMEPAD2//////////////////////////////////////////////////////

        upper.resetPos();
        upper.resetSlide();

        waitForStart();

        while(opModeIsActive()) {
//            if(gamepad2.dpad_up){
//                drive.setMotorPowers(0.5,0.5,0.5,0.5);
//            }
//            if(gamepad2.dpad_down){
//                drive.setMotorPowers(-0.5,-0.5,-0.5,-0.5);
//            }
//            if(gamepad2.dpad_left){
//                drive.setMotorPowers(-0.5,0.5,0.5,-0.5);
//            }
//            if(gamepad2.dpad_right){
//                drive.setMotorPowers(0.5,-0.5,-0.5,0.5);
//            }


            if (intakeFar.toTrue()) {
                upper.intakeFar();
            }
            if (intakeNear.toTrue()) {
                upper.intakeNear();
            }
            if (resetPos.toTrue()) {
                upper.resetPos();
            }
            if (releaseHigh.toTrue()) {

                upper.releaseHigh();
            }

            if (gamepad1.right_bumper) {
                upper.rollIn();
            } else if (gamepad1.left_bumper) {
                upper.rollOut();
            } else {
                upper.rollStop();
            }

            if (grabOpen.toTrue()) {
                upper.grabOpen();
            }
            if (grabClose.toTrue()) {
                upper.grabClose();
            }
            if (wristDrop.toTrue()) {
                upper.wristDrop();
            }
            if (wristIntake.toTrue()) {
                upper.wristIntake();
            }

            upper.update();
            telemetry.addData("arm: ", upper.getArmPosition());
            telemetry.addData("slideL: ", upper.getSlideLeftPosition());
            telemetry.addData("slideR: ", upper.getSlideRightPosition());
            telemetry.update();
            XCYBoolean.bulkRead();

        }

    }

    public static enum Sequences {
        FAR_INTAKE,
        NEAR_INTAKE,
        GET_HP,
        HIGH_CHAMBER,
        HIGH_NET
    }

// All of this is very bad and very experimental and also untested
    private void drive_period() {
        if (sequence == Sequences.NEAR_INTAKE || sequence == Sequences.FAR_INTAKE) {
            double x = -gamepad2.left_stick_y * 0.35 + -gamepad2.right_stick_y * 0.65;
            double y = -gamepad2.left_stick_x * 0.35 + -gamepad2.right_stick_x * 0.65;
            double turn_val = (gamepad2.left_trigger - gamepad2.right_trigger);
            Pose2d power = (new Pose2d(x, y, turn_val * 1)).times(1); //global drive turn ration times global drive power
            drive.setGlobalPower(power, 1,1); // x_static_compensation, y_static_compensation
        } else {
            double x = -gamepad1.left_stick_y * 0.35 + -gamepad1.right_stick_y * 0.65;
            double y = -gamepad1.left_stick_x * 0.35 + -gamepad1.right_stick_x * 0.65;
            double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
            Vector2d fast_stick = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
            double corrected_rad = fast_stick.angle() - current_pos.getHeading();
            while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
            while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
//            if (Math.abs(corrected_rad) < Math.PI / 5) {
//                double div = clamp(
//                        Math.toDegrees(corrected_rad) / 20, 1)
//                        * max_turn_assist_power * fast_stick.norm();
//                turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
//            }
            Pose2d power = (new Pose2d(x, y, turn_val * 1)).times(1);
            drive.setGlobalPower(power, 1, 1);
        }
        drive.update();
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        current_pos = drive.getPoseEstimate();
//        period_time_sec = time.seconds() - last_time_sec;
//        telemetry.addData("elapse time", period_time_sec * 1000);
//        last_time_sec = time.seconds();
        telemetry.update();
    }
}
