package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.TeleOpDrive;
import org.firstinspires.ftc.teamcode.references.*;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.uppersystems.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name = "16093 Command TeleOp")
public class CommandTeleOpTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    TeleOpDrive drive;
    Pose2d current_pos;
    Runnable update;
    //Runnable update;

    ArmSubsystem arm;
    SlideSubsystem slide;
    ClawSubsystem claw;
    GrabSubsystem grab;
    IntakeSubsystem intake;
    WristSubsystem wrist;

    boolean initComplete = false;

    // Modes for system control
    int driveMode = 0; // 0: POV mode; 1: Field-centric mode
    public static int slideMode=0;//1: setpower
    public static int armMode = 0;//1: setpower
    int wristPos=0;//0:up;1:down
    boolean intakeAct = false;
    double slideOpenloopConst = 0.3;

    double intakePosition = SSValues.CONTINUOUS_STOP; // Intake servo initial position

    XCYBoolean releaseSample;
    XCYBoolean changeClaw;
    XCYBoolean wristHeightSwitch;
    XCYBoolean resetOdo;

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void initialize(){

        CommandScheduler.getInstance().reset();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        Globals.IS_AUTO = false;
        Globals.stopIntaking();
        Globals.stopScoring();

        drive = new TeleOpDrive();
        drive.setUp(hardwareMap);
        robot.init(hardwareMap);

        arm = robot.getInstance().armSubsystem;
        slide = robot.getInstance().slideSubsystem;
        claw = robot.getInstance().clawSubsystem;
        grab = robot.getInstance().grabSubsystem;
        intake = robot.getInstance().intakeSubsystem;
        wrist = robot.getInstance().wristSubsystem;

        slide.resetSlide();
        arm.resetArmEncoder();

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new ResetFromIntakeCommand(), //if intaking
                                        new ConditionalCommand(
                                                new ResetFromScoringCommand(), //if scoring
                                                new SequentialCommandGroup(
                                                        new ArmCommand(SSValues.ARM_DOWN),
                                                        new WristCommand(SSValues.SLIDE_MIN)
                                                ), //if run
                                                () -> Globals.IS_SCORING
                                        ),
                                        () -> Globals.IS_INTAKING
                                ))
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::setIntakeSpin, intake))
                .whenReleased(new InstantCommand(intake::stopIntake, intake));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::setIntakeSpinOpposite, intake))
                .whenReleased(new InstantCommand(intake::stopIntake, intake));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand( //if intaking
                                        new SequentialCommandGroup(
                                                new InstantCommand(Globals::startScoring),
                                                new ResetFromIntakeCommand(),
                                                new WaitCommand(100),
                                                new ArmCommand(SSValues.ARM_UP),
                                                new SlideCommand(SSValues.SLIDE_MAX),
                                                new WristCommand(SSValues.WRIST_RELEASE)
                                        ),
                                        new ConditionalCommand( //if scoring
                                                new SequentialCommandGroup(
                                                        new WristCommand(SSValues.WRIST_RELEASE),
                                                        new WaitCommand(100),
                                                        new SlideCommand(SSValues.SLIDE_MAX),
                                                        new WristCommand(SSValues.WRIST_RELEASE)
                                                ),//if neither scoring nor intaking (i.e. run)
                                                new SequentialCommandGroup(
                                                        new InstantCommand(Globals::startScoring),
                                                        new ArmCommand(SSValues.ARM_UP),
                                                        new SlideCommand(SSValues.SLIDE_MAX),
                                                        new WristCommand(SSValues.WRIST_RELEASE)
                                                ),
                                                () -> Globals.IS_SCORING
                                        ),
                                        () -> Globals.IS_INTAKING
                                ))
                );

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand( //if intaking
                                        new SequentialCommandGroup(
                                                new InstantCommand(Globals::startScoring),
                                                new ResetFromIntakeCommand(),
                                                new WaitCommand(100),
                                                new ArmCommand(SSValues.ARM_UP),
                                                new WristCommand(SSValues.WRIST_RELEASE)
                                        ),
                                        new ConditionalCommand( //if scoring
                                                new SequentialCommandGroup(
                                                        new WristCommand(SSValues.WRIST_RELEASE),
                                                        new WaitCommand(100),
                                                        new SlideCommand(SSValues.SLIDE_MIN),
                                                        new WristCommand(SSValues.WRIST_RELEASE)
                                                ),//if neither scoring nor intaking (i.e. run)
                                                new SequentialCommandGroup(
                                                        new InstantCommand(Globals::startScoring),
                                                        new ArmCommand(SSValues.ARM_UP),
                                                        new WristCommand(SSValues.WRIST_RELEASE)
                                                ),
                                                () -> Globals.IS_SCORING
                                        ),
                                        () -> Globals.IS_INTAKING
                                ))
                );


        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(//if intaking
                                                new SlideCommand(SSValues.SLIDE_INTAKE_FAR)
                                        ),
                                        new ConditionalCommand( //if scoring
                                                new SequentialCommandGroup(
                                                        new ResetFromScoringCommand(),
                                                        new InstantCommand(Globals::startIntaking),
                                                        new WaitCommand(100),
                                                        new SlideCommand(SSValues.SLIDE_INTAKE_FAR),
                                                        new WristCommand(SSValues.WRIST_ABOVE_SAMPLES)
                                                ),//if neither scoring nor intaking (i.e. run)
                                                new SequentialCommandGroup(
                                                        new InstantCommand(Globals::startIntaking),
                                                        new SlideCommand(SSValues.SLIDE_INTAKE_FAR),
                                                        new WristCommand(SSValues.WRIST_ABOVE_SAMPLES)
                                                ),
                                                () -> Globals.IS_SCORING
                                        ),
                                        () -> Globals.IS_INTAKING
                                ))
                );

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(//if intaking
                                                new SlideCommand(SSValues.SLIDE_INTAKE_NEAR)
                                        ),
                                        new ConditionalCommand( //if scoring
                                                new SequentialCommandGroup(
                                                        new ResetFromScoringCommand(),
                                                        new InstantCommand(Globals::startIntaking),
                                                        new WaitCommand(100),
                                                        new SlideCommand(SSValues.SLIDE_INTAKE_NEAR),
                                                        new WristCommand(SSValues.WRIST_ABOVE_SAMPLES)
                                                ),//if neither scoring nor intaking (i.e. run)
                                                new SequentialCommandGroup(
                                                        new InstantCommand(Globals::startIntaking),
                                                        new SlideCommand(SSValues.SLIDE_INTAKE_NEAR),
                                                        new WristCommand(SSValues.WRIST_ABOVE_SAMPLES)
                                                ),
                                                () -> Globals.IS_SCORING
                                        ),
                                        () -> Globals.IS_INTAKING
                                ))
                );

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SlideCommand(SSValues.SLIDE_HIGH_CHAMBER_AIM))
                .whenReleased(new SlideCommand(SSValues.SLIDE_HIGH_CHAMBER_PLACE));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SlideCommand(SSValues.SLIDE_SLIGHTLY_LONGER).andThen(new InstantCommand(Globals::startScoring)));

        releaseSample = new XCYBoolean(() -> gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0);
        changeClaw = new XCYBoolean(() -> gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0);
        wristHeightSwitch = new XCYBoolean(() -> gamepad2.right_stick_button);
        resetOdo = new XCYBoolean(() -> gamepad1.a);

        initComplete = true;
    }

    public void run(){
        CommandScheduler.getInstance().run();
//        robot.clearBulkCache();
//        robot.read();
        robot.periodic();
//        robot.write();
        drive_period();

//            if((Math.abs(gamepad2.left_stick_y) > -0.1) && Globals.IS_INTAKING){
//                slide.setToCustomSlide();
//                if(intakeAct){
//                    slideOpenloopConst=0.2;
//                }
//                else{
//                    slideOpenloopConst=0.4;
//                }
//                if(gamepad2.left_stick_y > 0 && slide.getSlidesPosition() > 50){
//                    slide.setSlidesByPower(-gamepad2.left_stick_y*slideOpenloopConst);
//                }else if(gamepad2.left_stick_y < 0.1 && slide.getSlidesPosition() < SSValues.SLIDE_INTAKE_FAR+50){
//                    slide.setSlidesByPower(-gamepad2.left_stick_y*slideOpenloopConst);
//                }
//            }else{
//                slide.setToRegularSlide();
//            }

        if (gamepad2.options) {
            arm.setArmByPower(-1);
        }


        if (releaseSample.toTrue()) {
            grab.setGrabPos(SSValues.GRAB_OPEN);
        }

        if (Globals.IS_INTAKING && wristHeightSwitch.toTrue()) {
            if (wristPos == 0) {
                wrist.setWristPos(SSValues.WRIST_INTAKE);
                wristPos = 1;
            } else {
                wrist.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                wristPos = 0;
            }
        }

        //Claw opens/closes when driver 2 presses both triggers.
        if (changeClaw.toTrue()) {
            if (claw.isClawOpen()) {
                claw.setClawPos(ClawSubsystem.clawState.CLOSED);
            } else {
                claw.setClawPos(ClawSubsystem.clawState.CLOSED);
            }
        }

        if (resetOdo.toTrue()) {
            drive.resetOdo();
        }

        if(arm.getArmError() < 30){
            arm.setArmPower(0);
        }

        if(Math.abs(slide.getSlideError())<20){
            if(arm.getArmTargetPosition() == SSValues.ARM_UP) {
                if(slide.getSlideTargetPosition() == SSValues.SLIDE_MAX){
                    slide.setSlidePower(0.3);
                }else{
                    slide.setSlidePower(0.1);
                }
            } else {
                slide.setSlidePower(0.05);
            }
        }



        XCYBoolean.bulkRead();
        telemetry.addData("IS_SCORING", Globals.IS_SCORING);
        telemetry.addData("IS_INTAKING", Globals.IS_INTAKING);
        telemetry.addData("Arm Power", robot.armPower);
        telemetry.addData("Arm Position", robot.armPosition);
        telemetry.addData("Slide Power", robot.slidePower);
        telemetry.addData("Slide Position", robot.slidePosition);
        telemetry_M.update();
    }


    /////////////////////////// SUPPORT METHODS ////////////////////////////

    // Drive control handling for mecanum drive based on selected mode

    private void drive_period() {
        if(drive!= null){
            if (driveMode == 0) {
                drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, Globals.IS_SCORING, Globals.IS_INTAKING);
            } else {
                drive.setHeadingPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, Globals.IS_SCORING, Globals.IS_INTAKING);
            }
            drive.updateOdo();
        }
    }


}