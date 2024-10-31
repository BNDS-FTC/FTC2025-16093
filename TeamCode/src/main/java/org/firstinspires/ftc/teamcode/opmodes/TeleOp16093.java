package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.ArmAction;
import org.firstinspires.ftc.teamcode.uppersystems.SlideAction;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
import org.firstinspires.ftc.teamcode.uppersystems.Action;
import org.firstinspires.ftc.teamcode.uppersystems.WristAction;

import java.util.ArrayList;

@TeleOp(name = "16093TeleOp")
public class TeleOp16093 extends LinearOpMode {
    public SuperStructure upper;
    public NewMecanumDrive drive;
    private Sequences sequence;
    private Sequences previousSequence;
    private Pose2d current_pos;
    private Runnable update;
    public int mode=0;//when the sequence is changed, this integer turns to 1 to elicit further control
    public int armPosition;//the desired position of arm
    public int slidePosition;//the desired position of slide
    public double wristPosition;//the desired position of grab servo
    public ArrayList<Action> actionSequence = new ArrayList<>();
    public double intakePosition;//position of the intake servo

    @Override
    public void runOpMode() throws InterruptedException{
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


        ///////////////////////////GAMEPAD2//////////////////////////////////////////////////////

        ///////////////////////////INIT/////////////////////////////////////////////////////////

        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);
        upper.setArmByP(SSValues.ARM_DEFAULT, 0.5);

        sequence = Sequences.RUN;
        previousSequence = Sequences.RUN;
        waitForStart();
        upper.setIntake(0.5);
        logic_period();
        mode = 0;

        while(opModeIsActive()) {

            if (mode == 0) {
                if(releaseHigh.toTrue()){
                    mode = 1;
                    switchSequence(TeleOp16093.Sequences.HIGH_BASKET);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    if(previousSequence == Sequences.RUN){
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }else if(previousSequence == Sequences.INTAKE_FAR || previousSequence == Sequences.INTAKE_NEAR){
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }
                }
                if(resetPos.toTrue()){
                    mode = 1;
                    switchSequence(Sequences.RUN);
                    if(previousSequence == Sequences.INTAKE_FAR || previousSequence == Sequences.INTAKE_NEAR){
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                    }else if(previousSequence == Sequences.HIGH_BASKET){
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE_FAR));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                    }
                }
                if(intakeFar.toTrue()){
                    mode = 1;
                    switchSequence(Sequences.INTAKE_FAR);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    if(previousSequence == Sequences.RUN || previousSequence == Sequences.INTAKE_NEAR){
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_INTAKE_FAR));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE_FAR));
                    }else if(previousSequence == Sequences.HIGH_BASKET){
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE_FAR));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_INTAKE_FAR));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE_FAR));
                    }
                }
                if(intakeNear.toTrue()){
                    mode = 1;
                    switchSequence(Sequences.INTAKE_NEAR);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    if(previousSequence == Sequences.RUN){
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_INTAKE_NEAR));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIDDLE));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE_NEAR));
                    }else if(previousSequence == Sequences.INTAKE_FAR){
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIDDLE));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_INTAKE_NEAR));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE_NEAR));
                    }else if(previousSequence == Sequences.HIGH_BASKET){
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE_FAR));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_INTAKE_NEAR));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIDDLE));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE_NEAR));
                    }
                }

                if (gamepad1.right_bumper) {
                    intakePosition=SSValues.CONTINUOUS_SPIN;
                } else if (gamepad1.left_bumper) {
                    intakePosition=SSValues.CONTINUOUS_SPIN_OPPOSITE;
                } else {
                    if(intakePosition==SSValues.CONTINUOUS_SPIN_OPPOSITE){
                        upper.setIntake(SSValues.CONTINUOUS_STOP_OPPOSITE);
                    }
                    else {
                        upper.setIntake(SSValues.CONTINUOUS_STOP);
                    }
                }

            }


//            if(resetArm.toTrue()){
//                upper.resetArmEncoder();
//            }

            ///////////////////////////NOT GAMEPAD-RELATED//////////////////////////////////////////////

            if(mode == 1){
                buildSequence(actionSequence);
            }else{
                upper.setIntake(intakePosition);
            }
            drive_period();

            //upper.update();
            telemetry.addData("arm: ", upper.getArmPosition());
            telemetry.addData("slideL: ", upper.getSlideLeftPosition());
            telemetry.addData("slideR: ", upper.getSlideRightPosition());
            telemetry.addData("Arm Power",upper.getArmPower());
            telemetry.addData("Mode",mode);
            telemetry.addData("Current Sequence", sequence);
            telemetry.addData("Previous Sequence", previousSequence);
            telemetry.addData("Arm Diff",armPosition-upper.getArmPosition());
            telemetry.addData("Slide Diff",slidePosition-upper.getSlidePosition());

            telemetry.update();
            XCYBoolean.bulkRead();

        }

    }

    ///////////////////////////OUTSIDE THE LOOP//////////////////////////////////////////////////

    public void buildSequence(ArrayList<Action> actionSequence){
        int i = 0;
        while(i < actionSequence.size()){
            actionSequence.get(i).actuate();
            drive_period();
            upper.setIntake(intakePosition);
            if (actionSequence.get(i).isFinished()) {
                i++;
            }
        }
        actionSequence.clear();
        mode = 0;
    }

    public void switchSequence(Sequences s){
        previousSequence = sequence;
        sequence = s;
    }

    public static enum Sequences {
        RUN,
        INTAKE_FAR,
        INTAKE_NEAR,
        HIGH_BASKET
        //Etc.
    }

// This is just a normal mecanum drive
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
