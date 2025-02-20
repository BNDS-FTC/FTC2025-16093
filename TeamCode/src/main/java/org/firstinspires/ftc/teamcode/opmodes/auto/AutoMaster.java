package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.CancellableFinishConditionActionGroup;
import org.firstinspires.ftc.teamcode.actions.actioncore.FinishConditionActionGroup;
import org.firstinspires.ftc.teamcode.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.actions.TailAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.SequencerAction;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.TimerBoolean;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.actions.actioncore.Action;
import org.firstinspires.ftc.teamcode.actions.ArmAction;
import org.firstinspires.ftc.teamcode.actions.GrabAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.ParallelActionGroup;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.actions.WristAction;

import java.util.List;

@Config
public abstract class AutoMaster extends LinearOpMode {

    private NewMecanumDrive drive;
    protected SuperStructure upper;
    protected Runnable update;
    private List<LynxModule> allHubs;

    Pose2d startPos;

    double oldTime = 0;
    int loopCount = 0;
    static long startTime;
    boolean resetOnce = false;


    protected void initAuto(Pose2d start) throws InterruptedException{
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        startPos = start;
        startTime = System.currentTimeMillis();
        Action.clearActions();
        Action.setOpModeActive(()->opModeIsActive());

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new NewMecanumDrive(hardwareMap);
        drive.resetOdo();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(startPos);
        drive.setOpModeActive(()->opModeIsActive());
        drive.initialUpdate();
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(10));

        telemetry.addLine("init: superstructure");
        telemetry.update();
        upper = new SuperStructure(
                this,
                () -> {
                }, 0);

        TimerBoolean touchPressed = new TimerBoolean(() -> upper.getTouchSensorPressed(), ()->upper.getSequence() == SuperStructure.Sequences.RUN || upper.getSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN, 50);
//        XCYBoolean resetArm = new XCYBoolean(()->touchPressed.trueTimeReached());

        update = ()->{
            drive.update();
            upper.update();
            XCYBoolean.bulkRead();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
            loopCount ++;
//            telemetry.addData("Loops since start: ", loopCount);
//            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            if(drive.simpleMoveIsActivate){
                RobotLog.d("SimpleMove at "+drive.getPoseEstimate().toString()+" SimpleMove target "+drive.getSimpleMovePosition().toString());
            }
            if (!Action.actions.isEmpty()) {
                RobotLog.d("Running " + Action.showCurrentAction());
            }


//            telemetry.addData("Switch PID?", upper.slideTooHigh);
//            telemetry.addData("Start Time: ", drive.startTime);
//            telemetry.addData("Time since last MoveTo: ", drive.millisSinceMoveTo);
//            telemetry.addData("Current Pose", drive.getCurrentPoseAsString());
//            telemetry.addData("Target Pose",(drive.simpleMoveIsActivate)? drive.getSimpleMovePosition().toString() : "SimpleMove not activated");
//            telemetry.addData("Arm Position: ", upper.getArmPosition());
//            telemetry.addData("Arm Target: ", upper.getArmTargetPosition());
//            telemetry.addData("Slide Target: ", upper.getSlideTargetPosition());
//            telemetry.addData("Slide Error: ", upper.getSlideError());
//            telemetry.addData("Slide Power: ", upper.getSlidePower());
//            telemetry.addData("Current Sequence: ",upper.getSequence());
//            telemetry.addData("Current Time: ", System.currentTimeMillis()-startTime);
//            telemetry.addData("Touch Sensor Pressed?", upper.getTouchSensorPressed());
//            telemetry.addData("Touch time since true: ", touchPressed.getTimeSinceTrue());
//            telemetry.addData("Touch true time reached: ", touchPressed.trueTimeReached());
//
//
//            if(upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
//                telemetry.addData("Detected Sample Color", upper.colorOfSample());
//                telemetry.addData("Color Raw Values", upper.getColorRGBAValues(5).toString());
////            telemetry.addData("Is there a sample?", upper.colorSensorCovered());
//            }
//
//            telemetry.addLine(Action.showCurrentAction());
//            telemetry.update();
            if(!resetOnce){
                if (Math.abs(upper.getArmError()) < 30 && upper.getArmTargetPosition() == SSValues.ARM_DOWN-upper.armOffset) {
                    if (touchPressed.trueTimeReached() && (upper.getSequence() == SuperStructure.Sequences.RUN || upper.getSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN)){
                        if(Action.currentAction.returnType().equals("ArmAction")){
                            Action.currentAction.stop();
                        }
                        upper.armOffset = 0;
                        upper.resetArmEncoder();
                        resetOnce = true;
                    }
                }
            }
            //TODO: THIS DOESN'T WORK
//            if(drive.simpleMoveInDistress){
//                prepareForTeleOpWithoutMoving();
//            }
//            if(isStopRequested()){
//                Action.stopBuilding = true;
//            }

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        };

        drive.setUpdateRunnable(update);
        drive.setSwitchDrivePIDCondition(()->upper.slideTooHigh); //TODO: upper.armTooHigh;

        upper.resetSlide();
        upper.setGrabPos(SSValues.AUTO_GRAB_CLOSED);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setTailPos(SSValues.TAIL_DEFAULT);
        upper.setAscentState(SuperStructure.AscentState.ASCENT_DOWN);

//        upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);

        telemetry.addLine("init: complete");
        telemetry.update();


    }

    protected void setStartTime(){
        startTime = System.currentTimeMillis();
    }

    ///////////////////////////////////BLUE BASKET///////////////////////////////////////////////
    protected void reset(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        // Sequence actions based on last sequence
        if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
            upper.setGrabPos(SSValues.AUTO_GRAB_CLOSED);
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 10));
        } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.ASCENT || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
            upper.setGrabPos(SSValues.GRAB_DEFAULT);
            Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 100));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
            Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
            Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_RESET, 100));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        }
        Action.buildSequence(update);
    }

    protected void newResetFromHighChamber(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_RESET, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, (int)(0.8*SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO)));
        Action.buildSequence(update);
    }
    protected void newResetCompletelyFromHighChamber(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_RESET, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,SSValues.ARM_UP));
        Action.buildSequence(update);
    }
    protected Pose2d yellowPose;
    protected void resetAfterBlueBasketAndMoveToIntake(double xOffset, double headingOffset){
        yellowPose = new Pose2d(47.5+xOffset, 47, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.75);
        drive.moveTo(new Pose2d(50, 45, Math.toRadians(-135)), 20);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 100));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        drive.moveTo(yellowPose, 600, ()->Action.buildSequence(update));
    }

    protected void expResetAfterRedBasketAndMoveToIntake(double xOffset, double yOffset, double headingOffset){
        yellowPose = new Pose2d(-49+xOffset, -49+yOffset, Math.toRadians(90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);

        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, (int)(SSValues.SLIDE_MAX*0.75)));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 200));
        drive.moveTo(yellowPose, 100, ()->Action.buildSequence(update));
    }

    protected void expResetAfterBlueBasketAndMoveToIntake(double xOffset, double yOffset, double headingOffset){
        yellowPose = new Pose2d(47.5+xOffset, 47.5+yOffset, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);

        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, (int)(SSValues.SLIDE_MAX*0.75)));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 200));
        drive.moveTo(yellowPose, 100, ()->Action.buildSequence(update));
    }


    protected void newFirstMoveToBlueChamberPlace(){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.7);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new TailAction(upper, SSValues.TAIL_CHAMBER));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 800));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,150));
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,150));
        drive.moveTo(new Pose2d(-9, 36.2, Math.toRadians(90)),50, ()->Action.buildSequence(update));
//        drive.moveTo(new Pose2d(-10, 39.3, Math.toRadians(90)),50,()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,130));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN,100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void blueChamberPlaceFromWall(double xOffset, double yOffset){
//        drive.turnOnSwitchDrive(true);
        drive.setSimpleMoveTolerance(3,3, Math.toRadians(8));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 980));
        Action.buildSequence(update);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new TailAction(upper,SSValues.TAIL_CHAMBER));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO));
        drive.moveTo(new Pose2d(-10+xOffset, 40+yOffset, Math.toRadians(90)),100,()->{Action.buildSequence(update);});
        drive.moveTo(new Pose2d(-10+xOffset, 33.3+yOffset, Math.toRadians(90)),10, ()->drive.setSimpleMovePower(0.5));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,150));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        drive.turnOnSwitchDrive(false);
    }

    protected void redChamberPlaceFromWall(double xOffset, double yOffset){
//        drive.turnOnSwitchDrive(true);
        drive.setSimpleMoveTolerance(3,3, Math.toRadians(8));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 980));
        Action.buildSequence(update);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new TailAction(upper,SSValues.TAIL_CHAMBER));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO));
        drive.moveTo(new Pose2d(13+xOffset, -43.5+yOffset, Math.toRadians(-90)),20,()->{Action.buildSequence(update);});
        drive.moveTo(new Pose2d(13+xOffset, -33.5+yOffset, Math.toRadians(-90)),10, ()->drive.setSimpleMovePower(0.5));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,150));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        drive.turnOnSwitchDrive(false);
    }




    protected void intakeSpecimenFromGround(double xOffset, double yOffset){
        drive.setSimpleMovePower(0.95);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,600));
        Action.actions.add(new SlideAction(upper, (int)(SSValues.SLIDE_INTAKE_FAR*0.75),10, 0.8));
        drive.moveTo(new Pose2d(-20+xOffset, 45+yOffset, Math.toRadians(135)), 10, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,20,0.15),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);}));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED));
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.3));
//        Action.buildSequence(update);
//        delay(40);
//        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void intakeSpecimenFromBlueWall(double xOffset, double yOffset){
//        drive.turnOnSwitchDrive(false);
        drive.setSimpleMovePower(1);
        drive.setSimpleMoveTolerance(4,1,Math.toRadians(7));
        upper.switchSequence(SuperStructure.Sequences.INTAKE_SPECIMEN);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE_WALL_SPECIMEN));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_GET_WALL_SPECIMEN,400));
        drive.moveTo(new Pose2d(-33.5+xOffset,48.7+yOffset,Math.toRadians(90)), 50, ()->Action.buildSequence(update));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT,80));
        Action.actions.add(new SlideAction(upper,SSValues.SLIDE_INTAKE_WALL_SPECIMEN,10, 0.6));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED,100));
        Action.buildSequence(update);
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 50));
//        Action.buildSequence(update);
    }

    protected void intakeSpecimenFromRedWall(double xOffset, double yOffset){
//        drive.turnOnSwitchDrive(false);
        drive.setSimpleMovePower(1);
        drive.setSimpleMoveTolerance(4,1,Math.toRadians(7));
        upper.switchSequence(SuperStructure.Sequences.INTAKE_SPECIMEN);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE_WALL_SPECIMEN));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_GET_WALL_SPECIMEN,400));
        drive.moveTo(new Pose2d(34.5+xOffset,-48.9+yOffset,Math.toRadians(-90)), 10, ()->Action.buildSequence(update));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT,180));
        Action.actions.add(new SlideAction(upper,SSValues.SLIDE_INTAKE_WALL_SPECIMEN,10, 0.6));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED,80));
        Action.buildSequence(update);
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 50));
//        Action.buildSequence(update);
    }

    protected void newFirstMoveToRedChamberPlace(){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new TailAction(upper, SSValues.TAIL_CHAMBER));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 900));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,600));
        Action.actions.add(new SequencerAction(()->drive.setSimpleMovePower(0.5),0));
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,150));
        drive.moveTo(new Pose2d(9, -36.4, Math.toRadians(-90)),70, ()->Action.buildSequence(update));
//        drive.moveTo(new Pose2d(-10, 39.3, Math.toRadians(90)),50,()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,140));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN,100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.actions.add(new TailAction(upper, SSValues.TAIL_DEFAULT, 180));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void newRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER,600),new ArmAction(upper, SSValues.ARM_UP, 700)));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,10));
        drive.moveTo(new Pose2d(-10+xOffset, 37, Math.toRadians(90)),20,()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,100));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void newParkFromRedChamber(){
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,400));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN,100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,20));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(-20, 48, Math.toRadians(135)), 0, ()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,100));
        Action.buildSequence(update);
    }


    protected Pose2d blueBasket = new Pose2d(53.3, 54.5, Math.toRadians(-135));
    protected Pose2d redBasket = new Pose2d(-53.3, -56.3, Math.toRadians(45));

    protected void expFirstPutBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.65);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 0));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 90));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 700));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 90));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 90));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(54, 54, Math.toRadians(-130)), 200,()-> {
            Action.buildSequence(update);
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 90));
            Action.buildSequence(update);}
        );
//        sleep(400);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE_AUTO,200));
        Action.buildSequence(update);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE_AUTO,40));
        Action.actions.add(new IntakeAction(upper, SSValues.CONTINUOUS_SPIN_OPPOSITE, 10));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void expFirstPutRedBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.65);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 0));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 90));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 700));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 90));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 90));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(-53.5, -55, Math.toRadians(50)), 200,()-> {
            Action.buildSequence(update);
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 90));
            Action.buildSequence(update);}
        );
//        sleep(400);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE_AUTO,200));
        Action.buildSequence(update);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE_AUTO,40));
        Action.actions.add(new IntakeAction(upper, SSValues.CONTINUOUS_SPIN_OPPOSITE, 10));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putBlueBasketFromGround(double xOffset, double yOffset, double simpleMovePowerChange){
        blueBasket = new Pose2d(52.3+xOffset, 56.2+yOffset, Math.toRadians(-125));
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.3 + simpleMovePowerChange);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 800));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE_AUTO,340));
        drive.moveTo(blueBasket, 250,()->Action.buildSequence(update));
//        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        Action.actions.add(new IntakeAction(upper, SSValues.CONTINUOUS_SPIN_OPPOSITE, 20));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        sleep(150);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putRedBasketFromGround(double xOffset, double yOffset, double simpleMovePowerChange){
        blueBasket = new Pose2d(-52.3+xOffset, -56.2+yOffset, Math.toRadians(60));
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.3 + simpleMovePowerChange);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 800));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE_AUTO,340));
        drive.moveTo(blueBasket, 250,()->Action.buildSequence(update));
//        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        Action.actions.add(new IntakeAction(upper, SSValues.CONTINUOUS_SPIN_OPPOSITE, 20));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        sleep(150);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putBlueBasketFromSubmersible(double xOffset, double yOffset, double degreeOffset){
        blueBasket = new Pose2d(52+xOffset, 56+yOffset, Math.toRadians(-110));
//        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
//        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
//        drive.setSimpleMovePower(0.4 + simpleMovePowerChange);
//        drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 600);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 400));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 900));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        drive.moveTo(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(-110)), 0,()->Action.buildSequence(update));
//        drive.moveTo(new Pose2d(33, 20, Math.toRadians(-135)),0, ()->Action.buildSequence(update));
        drive.setSimpleMovePower(0.7);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, (int)(SSValues.ARM_UP*0.3)));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, SSValues.SLIDE_MAX));
        Action.actions.add(new SequencerAction(()->drive.setSimpleMovePower(0.4),0));
        drive.setSimpleMoveTolerance(1.5, 1.5, Math.toRadians(7));
        drive.moveTo(new Pose2d(blueBasket.getX()-2, blueBasket.getY()-2, Math.toRadians(-110+degreeOffset)), 0,()->{Action.buildSequence(update);});
        drive.moveTo(blueBasket, 50);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE_EXTRA,340));
        Action.actions.add(new IntakeAction(upper, SSValues.CONTINUOUS_SPIN_OPPOSITE));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delay(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putRedBasketFromSubmersible(double xOffset, double yOffset, double degreeOffset, double simpleMovePowerChange){
        blueBasket = new Pose2d(-52.3+xOffset, -56.2+yOffset, Math.toRadians(60));
//        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
//        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
//        drive.setSimpleMovePower(0.4 + simpleMovePowerChange);
//        drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 600);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 400));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 900));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        drive.moveTo(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(-110)), 0,()->Action.buildSequence(update));
//        drive.moveTo(new Pose2d(33, 20, Math.toRadians(-135)),0, ()->Action.buildSequence(update));
        drive.setSimpleMovePower(0.7);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, (int)(SSValues.ARM_UP*0.3)));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 700));
        Action.actions.add(new SequencerAction(()->drive.setSimpleMovePower(0.4),0));
        drive.setSimpleMoveTolerance(1.5, 1.5, Math.toRadians(7));
        drive.moveTo(new Pose2d(redBasket.getX(), redBasket.getY(), Math.toRadians(70+degreeOffset)), 50,()->{Action.buildSequence(update);drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);});
//        drive.moveTo(blueBasket, 50,()->{drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);});
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE_EXTRA,340));
        Action.actions.add(new IntakeAction(upper, SSValues.CONTINUOUS_SPIN_OPPOSITE));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        delay(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }




    // Method to move the robot with proportional deceleration and motor power adjustment
    private void moveToWithDeceleration(Pose2d target, double speed) {
        // Get the current position of the robot
        Pose2d currentPose = drive.getPoseEstimate();

        // Calculate the distance to the target
        double targetY = target.getY();
        double currentY = currentPose.getY();

        double distanceToTarget = Math.abs(targetY - currentY);

        // Define the maximum speed
        double maxSpeed = speed;  // You can adjust the max speed here

        // Calculate the speed using proportional deceleration based on remaining distance
        double adjustedSpeed = calculateSpeed(distanceToTarget, maxSpeed);

        // Convert the adjusted speed into motor power
        double motorPower = speedToMotorPower(adjustedSpeed);

        // Move to the target with adjusted motor power
        drive.setMotorPowers(motorPower, motorPower,motorPower,motorPower);
        drive.moveTo(new Pose2d(target.getX(),
                target.getY(),
                target.getHeading()),0);
    }

    // Function to calculate the robot's speed based on distance to the target
    private double calculateSpeed(double distanceToTarget, double maxSpeed) {
        // Proportional deceleration: speed decreases as distance decreases
        double speed = maxSpeed * (distanceToTarget / 5);  // 5 is the maximum distance to decelerate (you can adjust this value)

        // Ensure the speed doesn't go below a minimum threshold
        if (speed < 0.1) {  // You can adjust the minimum speed
            speed = 0.1;
        }

        return speed;
    }

    // Convert speed to motor power (scale to -1 to 1 range)
    private double speedToMotorPower(double speed) {
        // Assuming the max speed is normalized to 1.0, and we want the motor power to be proportional to this speed
        // Ensure the motor power is within -1.0 and 1.0
        double motorPower = speed;  // Since speed is normalized between 0 and maxSpeed, motor power can directly match it

        // Optional: Apply a deadzone to avoid jittering at low speeds (you can tune this value)
        if (Math.abs(motorPower) < 0.1) {
            motorPower = 0;  // Deadzone to prevent small motor movements
        }

        return motorPower;
    }

    protected void VexpPushTwoRedSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        drive.moveTo(new Pose2d(32, -40, Math.toRadians(-90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        drive.moveWithDrift(new Pose2d(34, -12, Math.toRadians(-90)),
                new Pose2d(43, -12, Math.toRadians(-90)),
                new Pose2d(43, -48, Math.toRadians(-90)),
                new Pose2d(43, -11, Math.toRadians(-90)),
                new Pose2d(54, -11, Math.toRadians(-90)),
                new Pose2d(55, -49, Math.toRadians(-90)),
                new Pose2d(54, -11, Math.toRadians(-90)),
                new Pose2d(59.6, -11, Math.toRadians(-90)),
                new Pose2d(59.6, -49, Math.toRadians(-90)));
    }

    protected void VexpPushTwoBlueSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        drive.moveTo(new Pose2d(-34, 40, Math.toRadians(90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        drive.moveTo(new Pose2d(-34, 15, Math.toRadians(90)), 20,()->Action.buildSequence(update));
        drive.moveWithDrift(new Pose2d(-46, 17, Math.toRadians(90)),
                new Pose2d(-46, 48, Math.toRadians(90)),
                new Pose2d(-46, 17, Math.toRadians(90)),
                new Pose2d(-54, 17, Math.toRadians(90)),
                new Pose2d(-55, 48, Math.toRadians(90)),
                new Pose2d(-54, 17, Math.toRadians(90)),
                new Pose2d(-59, 17, Math.toRadians(90)),
                new Pose2d(-59, 45, Math.toRadians(90)));
    }



    protected void expGetYellowSamples(){
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(3));
        drive.setSimpleMovePower(0.65);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,0));
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_YELLOW,20,0.4),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);}));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setGrabPos(SSValues.GRAB_CLOSED);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.buildSequence(update);
    }



    private Pose2d bluePose;

    protected void getSamplesFromSubmersibleBlue(double offset){

        resetAndGoToBlueSubmersible(0,offset);

        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.35),
                ()-> upper.colorOfSample().equals("blue")||upper.colorOfSample().equals("yellow"),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);},
                ()->{drive.moveTo(new Pose2d(drive.getCurrentPose().getX(),drive.getCurrentPose().getY()+2, Math.toRadians(180)),10);
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    tryAgainAtBlueSubmersible();}));
        drive.moveTo(new Pose2d(drive.getSimpleMovePosition().getX(), drive.getSimpleMovePosition().getY(), Math.toRadians(170)),100, ()->Action.buildSequence(update));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

    }

    protected void getSamplesFromSubmersibleRed(double offset){

        resetAndGoToRedSubmersible(0,offset);

        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.35),
                ()-> upper.colorOfSample().equals("red")||upper.colorOfSample().equals("yellow"),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setSlidePower(0);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);},
                ()->{drive.moveTo(new Pose2d(drive.getCurrentPose().getX(),drive.getCurrentPose().getY()+2, Math.toRadians(0)),10);
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    tryAgainAtRedSubmersible();}));
        drive.moveTo(new Pose2d(drive.getSimpleMovePosition().getX(), drive.getSimpleMovePosition().getY(), Math.toRadians(-10)),100, ()->Action.buildSequence(update));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

    }

    protected void getSamplesFromSubmersibleBlueWithEmergencyAscent(double offset) {
        CancellableFinishConditionActionGroup grabBlueFromSubmersible = new CancellableFinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR, 10, 0.35),
                () -> upper.colorOfSample().equals("blue") || upper.colorOfSample().equals("yellow"),
                () -> (System.currentTimeMillis() - startTime > 28.5 * 1000),
                () -> {
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);
                },
                () -> {
                    tryAgainAtBlueSubmersible();
                },
                () -> {
                    Action.clearActions();
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_LONGER));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1));
                    drive.moveTo(new Pose2d(20, 10, Math.toRadians(180)), 100, () -> Action.buildSequence(update));
                    while (opModeIsActive()) {
                    }
                });

        resetAndGoToBlueSubmersible(50, offset);

        Action.actions.add(grabBlueFromSubmersible);
        drive.moveTo(new Pose2d(drive.getSimpleMovePosition().getX(), drive.getSimpleMovePosition().getY(), Math.toRadians(195)), 100, () -> Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

    }

    protected void getSamplesFromSubmersibleRedWithEmergencyAscent(double offset) {
        CancellableFinishConditionActionGroup grabRedFromSubmersible = new CancellableFinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR, 10, 0.35),
                () -> upper.colorOfSample().equals("red") || upper.colorOfSample().equals("yellow"),
                () -> (System.currentTimeMillis() - startTime > 28.5 * 1000),
                () -> {
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);
                },
                () -> {
                    tryAgainAtRedSubmersible();
                },
                () -> {
                    Action.clearActions();
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_LONGER));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1));
                    drive.moveTo(new Pose2d(20, 10, Math.toRadians(0)), 100, () -> Action.buildSequence(update));
                    while (opModeIsActive()) {
                    }
                });

        resetAndGoToRedSubmersible(50, offset);

        Action.actions.add(grabRedFromSubmersible);
        drive.moveTo(new Pose2d(drive.getSimpleMovePosition().getX(), drive.getSimpleMovePosition().getY(), Math.toRadians(5)), 100, () -> Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

    }


    private void resetAndGoToBlueSubmersible(int correctTime, double offset) {
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(6));
        drive.setSimpleMovePower(1);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 200));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 750));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 750));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 200));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        drive.moveTo(new Pose2d(40, 14+offset, Math.toRadians(-135)), 0, () -> Action.buildSequence(update));
        drive.moveTo(new Pose2d(blueBasket.getX(), 16, Math.toRadians(-180)), 0, () -> Action.buildSequence(update));
        drive.moveTo(new Pose2d(18, 16 - offset, Math.toRadians(180 + offset)), 0, () -> {
                    Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT, 300));
                    Action.actions.add(new SequencerAction(() -> drive.setSimpleMovePower(0.4), 700));
                    Action.buildSequence(update);
                    upper.setWristPos(SSValues.WRIST_INTAKE);
                    upper.setIntake(SSValues.CONTINUOUS_SPIN);
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                }
        );
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    private void resetAndGoToRedSubmersible(int correctTime, double offset){
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(6));
        drive.setSimpleMovePower(1);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 200));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 750));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN+100, 750));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 200));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        drive.moveTo(new Pose2d(40, 14+offset, Math.toRadians(-135)), 0, () -> Action.buildSequence(update));
        drive.moveTo(new Pose2d(redBasket.getX(), -16, Math.toRadians(0)),0, ()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(-30, -6, Math.toRadians(0+offset)), 0, () -> {//x=-26.5,y=-8-offset
                    Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT, 300));
                    Action.actions.add(new SequencerAction(()->drive.setSimpleMovePower(0.4), 700));
                    Action.buildSequence(update);
                    upper.setWristPos(SSValues.WRIST_INTAKE);
                    upper.setIntake(SSValues.CONTINUOUS_SPIN);
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                }
        );
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void tryAgainAtBlueSubmersible(){
        FinishConditionActionGroup reachAgainAtBlueSubmersible = new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.24),
                ()->upper.colorOfSample().equals("blue")||upper.colorOfSample().equals("yellow"),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);
                    upper.setWristPos(SSValues.WRIST_DEFAULT);},
                ()->upper.setIntake(SSValues.CONTINUOUS_STOP));

        FinishConditionActionGroup hangIfNearEnd = new FinishConditionActionGroup(reachAgainAtBlueSubmersible,
                ()-> System.currentTimeMillis()-startTime > 29500,
                ()->{Action.clearActions();
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER));
                    Action.buildSequence(update);
                    while (opModeIsActive()) {}},
                ()->{});

        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveTo(new Pose2d(drive.getCurrentPose().getX(),drive.getCurrentPose().getY()+5, Math.toRadians(180)), 50, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(hangIfNearEnd);
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void tryAgainAtRedSubmersible(){
        FinishConditionActionGroup reachAgainAtRedSubmersible = new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.24),
                ()->upper.colorOfSample().equals("red")||upper.colorOfSample().equals("yellow"),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);
                    upper.setWristPos(SSValues.WRIST_DEFAULT);},
                ()->upper.setIntake(SSValues.CONTINUOUS_STOP));

        FinishConditionActionGroup hangIfNearEnd = new FinishConditionActionGroup(reachAgainAtRedSubmersible,
                ()-> System.currentTimeMillis()-startTime > 29500,
                ()->{Action.clearActions();
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER));
                    Action.buildSequence(update);
                    while (opModeIsActive()) {}},
                ()->{});

        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveTo(new Pose2d(drive.getCurrentPose().getX(),drive.getCurrentPose().getY()-5, Math.toRadians(0)), 50, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(hangIfNearEnd);
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }


    protected Pose2d lastBlueBasketSample = new Pose2d(53.8, 47, Math.toRadians(-60));
    protected Pose2d lastRedBasketSample = new Pose2d(-54,-46.0,Math.toRadians(141));
    protected void moveAndIntakeLastBasketSampleBlue(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(5));
        drive.setSimpleMovePower(0.3);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 950));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 200));
//        moveToGetLastYellowSample();
        drive.moveTo(lastBlueBasketSample, 150, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE,20,0.5),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);}));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void moveAndIntakeLastBasketSampleRed(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(5));
        drive.setSimpleMovePower(0.3);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 950));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 200));
//        moveToGetLastYellowSample();
        drive.moveTo(lastRedBasketSample, 150, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE,20,0.5),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);}));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);

    }

    protected void autoResetArmTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.buildSequence(update);
        upper.setArmTargetPosition(SSValues.ARM_DOWN-SSValues.AUTO_ARM_OFFSET);
    }

    public static  double testPIDx = 0, testPIDy = 0, testPIDheading = 90;
    public static  double targetPIDx = 0, targetPIDy = 40, targetPIDheading = 90;
    Pose2d[] poses = {new Pose2d(targetPIDx,targetPIDy,Math.toRadians(targetPIDheading)),new Pose2d(testPIDx,testPIDy,Math.toRadians(testPIDheading))};
    private Pose2d currentPose;
    private int poseCount = 0;
    protected void testAutoPID(){
        drive.setSimpleMovePower(0.5);
        if(!drive.isBusy()){
            if(poseCount < poses.length){
                currentPose = poses[poseCount];
                drive.moveTo(currentPose,200);
                poseCount++;
            }else{
                poseCount = 0;
            }
        }
    }

    protected void autoArmTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.buildSequence(update);
    }


    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            update.run();
        }
    }
}