package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.Action;
import org.firstinspires.ftc.teamcode.uppersystems.ArmAction;
import org.firstinspires.ftc.teamcode.uppersystems.SlideAction;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
import org.firstinspires.ftc.teamcode.uppersystems.WristAction;

import java.util.ArrayList;

@TeleOp
public class SequenceTest extends LinearOpMode {
    public SuperStructure upper;
    public NewMecanumDrive drive;
    private TeleOp16093.Sequences sequence;
    private TeleOp16093.Sequences previousSequence;
    private Pose2d current_pos;
    private Runnable update;
    public int mode=0; //O -> when the system is accepting new gamepad inputs. 1 -> when an input has been passed & is running.
    public ArrayList<Action> actionSequence = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        SuperStructure upper = new SuperStructure(
                this,
                () -> {
                    logic_period();
                    drive_period();
                });
        drive = new NewMecanumDrive();
        drive.setUp(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.update();

//        update = ()
//        upper.setUpdateRunnable(update);

        ///////////////////////////GAMEPAD1//////////////////////////////////////////////////////
        XCYBoolean intakeFar =new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean intakeNear = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean resetPos = new XCYBoolean(()->gamepad1.left_stick_button);
        XCYBoolean releaseHigh = new XCYBoolean(()->gamepad1.y);
        XCYBoolean intakeIn = new XCYBoolean(()->gamepad1.right_bumper);
        XCYBoolean intakeOut = new XCYBoolean(()->gamepad1.left_bumper);
        XCYBoolean grabOpen = new XCYBoolean(()->gamepad1.a);
        XCYBoolean grabClose = new XCYBoolean(()->gamepad1.b);
        XCYBoolean wristIntake = new XCYBoolean(()->gamepad1.dpad_left);
        XCYBoolean wristDrop = new XCYBoolean(()->gamepad1.dpad_right);
        XCYBoolean initPos = new XCYBoolean(()->gamepad1.start);
        XCYBoolean armUpSimple = new XCYBoolean(()->gamepad1.back);
        XCYBoolean releaseSpecimen = new XCYBoolean(()->gamepad1.right_trigger>0&&gamepad1.left_trigger>0);

        XCYBoolean resetArm = new XCYBoolean(()-> upper.getTouchSensorPressed());

        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);
        upper.setArmByP(SSValues.ARM_DEFAULT, 0.5);

        waitForStart();

        while(opModeIsActive()){
            if (mode == 0) {
                if(releaseHigh.toTrue()){
                    mode = 1;
                    switchSequence(TeleOp16093.Sequences.HIGH_BASKET);
                    if(previousSequence == TeleOp16093.Sequences.RUN){
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }else if(previousSequence == TeleOp16093.Sequences.INTAKE_FAR){
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }
                }
                if(resetPos.toTrue()){
                    mode = 1;
                    switchSequence(TeleOp16093.Sequences.RUN);
                    if(previousSequence == TeleOp16093.Sequences.INTAKE_FAR){
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                    }else if(previousSequence == TeleOp16093.Sequences.HIGH_BASKET){
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                    }
                }
                if(intakeFar.toTrue()){
                    mode = 1;
                    switchSequence(TeleOp16093.Sequences.INTAKE_FAR);
                    if(previousSequence == TeleOp16093.Sequences.RUN){
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    }else if(previousSequence == TeleOp16093.Sequences.HIGH_BASKET){
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    }
                }
            }

            drive_period();

            if(mode == 1){
                buildSequence(actionSequence);
            }

            //upper.update();
            telemetry.addData("arm: ", upper.getArmPosition());
            telemetry.addData("slideR: ", upper.getSlideRightPosition());
            telemetry.addData("Arm Power",upper.getArmPower());
            telemetry.addData("Mode",mode);
            //This is missing error telemetry.

            telemetry.update();
            XCYBoolean.bulkRead();

        }
    }

    public void buildSequence(ArrayList<Action> actionSequence){
        int i = 0;
        while(i < actionSequence.size()){
            actionSequence.get(i).actuate();
            drive_period();
            if (actionSequence.get(i).isFinished()) {
                i++;
            }
        }
        actionSequence.clear();
        mode = 0;
    }

    public void switchSequence(TeleOp16093.Sequences s){
        previousSequence = sequence;
        sequence = s;
    }

    // All of this is very bad and very experimental and also untested
    private void drive_period() {
        drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y,gamepad1.right_stick_x, sequence);
//        if (sequence == Sequences.NEAR_INTAKE || sequence == Sequences.FAR_INTAKE) {
//            double x = -gamepad1.left_stick_y * 0.35 + -gamepad1.right_stick_y * 0.65;
//            double y = -gamepad1.left_stick_x * 0.35 + -gamepad1.right_stick_x * 0.65;
//            double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
//            Pose2d power = (new Pose2d(x, y, turn_val * 1)).times(1); //global drive turn ration times global drive power
//            drive.setGlobalPower(power, 1,1); // x_static_compensation, y_static_compensation
//        } else {
//            double x = -gamepad1.left_stick_y * 0.35 + -gamepad1.right_stick_y * 0.65;
//            double y = -gamepad1.left_stick_x * 0.35 + -gamepad1.right_stick_x * 0.65;
//            double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
//            Vector2d fast_stick = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
//            double corrected_rad = fast_stick.angle() - current_pos.getHeading();
//            while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
//            while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
////            if (Math.abs(corrected_rad) < Math.PI / 5) {
////                double div = clamp(
////                        Math.toDegrees(corrected_rad) / 20, 1)
////                        * max_turn_assist_power * fast_stick.norm();
////                turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
////            }
//            Pose2d power = (new Pose2d(x, y, turn_val * 1)).times(1);
//            drive.setGlobalPower(power, 1, 1);
//        }
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