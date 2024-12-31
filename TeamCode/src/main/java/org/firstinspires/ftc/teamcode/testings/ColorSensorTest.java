package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.ArmAction;
import org.firstinspires.ftc.teamcode.actions.GrabAction;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.actions.WristAction;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;

import java.util.Objects;

@TeleOp(name="color sensor test", group="Testing")
public class ColorSensorTest extends LinearOpMode {
    private final double POWER=0.5;
    public void runOpMode(){
        SuperStructure upper;
        upper = new SuperStructure(
                this,
                () -> {
                }, 0);
        NewMecanumDrive drive;
        drive = new NewMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            drive.setHeadingPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, upper.getSequence());
            if(gamepad1.b){
                upper.setIntake(SSValues.CONTINUOUS_SPIN);
                Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN, 200));
                Action.buildSequence(()->{});
            }
            if(gamepad1.y){
                upper.setIntake(SSValues.CONTINUOUS_STOP);
            }
            if(gamepad1.x){
                upper.setIntake(SSValues.CONTINUOUS_SPIN);
                Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
                Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT, 50));
                Action.actions.add(new SlideAction(upper,SSValues.SLIDE_MIN,50));
                Action.buildSequence(()->{});

                while(!(Objects.equals(upper.colorOfSample(), "red")||Objects.equals(upper.colorOfSample(), "yellow"))){
                    if(gamepad1.y)break;
                    upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, POWER);
                    telemetry.addData("RGBA",upper.getColorRGBAValues(5));
                    telemetry.addData("res",upper.colorOfSample());
                    telemetry.addData("Slide Pos",upper.getSlidesPosition());
                    telemetry.addData("Statue","Slide running");
                    telemetry.update();
                }
                String s=upper.colorOfSample();
                upper.setIntake(SSValues.CONTINUOUS_STOP);
                upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, 0);
                Action.actions.add(new GrabAction(upper, SSValues.AUTO_GRAB_CLOSED, 200));
                Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                Action.buildSequence(()->{});
                sleep(300);
                Action.actions.add(new SlideAction(upper,SSValues.SLIDE_MIN,20));
                Action.buildSequence(()->{});
                if(upper.colorOfSample()!=s){
                    telemetry.addData("Statue","retrying");
                }
            }
            telemetry.addData("RGBA",upper.getColorRGBAValues(5));
            telemetry.addData("res",upper.colorOfSample());
            telemetry.addData("Slide Pos",upper.getSlidesPosition());
            telemetry.addData("Statue","Normal");
            telemetry.update();
        }
    }
}
