package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
@Config
public abstract class OpenLoopTeleOp16093 extends LinearOpMode {
    public double slideOpenloopConst = 1;
    public double armOpenloopConst = 1;
    @Override
    public void runOpMode(){
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        SuperStructure upper = new SuperStructure(this,()->{},0);
        while(opModeIsActive()){
            upper.setSlidesByPower(SSValues.SLIDE_MIN, -gamepad2.left_stick_y * slideOpenloopConst);
            upper.setArmByPower(SSValues.ARM_DOWN,-gamepad2.right_stick_y * armOpenloopConst);
            drive.setFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, null);
            drive.update();

            telemetry.addData("Arm Position: ", upper.getArmPosition());
            telemetry.addData("Slide Position: ", upper.getSlidesPosition());
            telemetry.addLine("");
            telemetry.addData("Arm Power", upper.getArmPower());
            telemetry.addData("Slide Power:", upper.getSlidePower());

            telemetry.update();

            upper.update();
        }
    }
}