package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.uppersystems.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name = "Command TeleOp")
public class CommandTeleOpTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    TeleOpDrive drive;
    SuperStructure upper;
    Pose2d current_pos;
    Runnable update;
    //Runnable update;

    // Modes for system control
    int driveMode = 0; // 0: POV mode; 1: Field-centric mode
    public static int slideMode=0;//1: setpower
    public static int armMode = 0;//1: setpower
    int wristPos=0;//0:up;1:down
    boolean intakeAct = false;
    double slideOpenloopConst = 0.3;

    double intakePosition = SSValues.CONTINUOUS_STOP; // Intake servo initial position

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        Globals.IS_AUTO = false;
        Globals.stopIntaking();
        Globals.stopScoring();

        robot.init(hardwareMap);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new ResetFromIntakeCommand(),
                                        new ConditionalCommand(
                                                new ResetFromScoringCommand(),
                                                new WaitCommand(0),//?????
                                                () -> Globals.IS_SCORING
                                        ),
                                        () -> Globals.IS_INTAKING
                                ))
                );

        IntakeSubsystem intake = robot.getInstance().intakeSubsystem;
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(() -> intake.setIntake(SSValues.CONTINUOUS_SPIN), intake))
                .whenReleased(new InstantCommand(intake::stopIntake, intake));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(Globals::startScoring),
                        new ArmCommand(SSValues.ARM_UP),
                        new SlideCommand(SSValues.SLIDE_MAX),
                        new WristCommand(SSValues.WRIST_RELEASE)
                ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(Globals::startScoring),
                        new ArmCommand(SSValues.ARM_UP),
                        new SlideCommand(SSValues.SLIDE_MIN),
                        new WristCommand(SSValues.WRIST_RELEASE)
                ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(Globals::startIntaking),
                        new WristCommand(SSValues.WRIST_DEFAULT),
                        new WaitCommand(100),
                        new SlideCommand(SSValues.SLIDE_MAX),
                        new WristCommand(SSValues.WRIST_ABOVE_SAMPLES)
                ));
    }

    public void run(){
        CommandScheduler.getInstance().run();
//        robot.clearBulkCache();
//        robot.read();
        robot.periodic();
//        robot.write();
        drive_period();
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
        if(upper!= null){
            if (driveMode == 0) {
                drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, upper.getSequence());
            } else {
                drive.setHeadingPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, upper.getSequence());
            }
            drive.updateOdo();
        }
    }

}