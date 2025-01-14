package org.firstinspires.ftc.teamcode.testings;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.references.SSValues;

@TeleOp
@Config
public class ArmAdjustment extends LinearOpMode{
    public static double armPowerUp = 1;
    public static double armPowerDown;
    public static double armMinPower = 0.35;
    public static double coefficient = 1.4;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx armUp = hardwareMap.get(DcMotorEx.class, "armUp");
        DcMotorEx armDown = hardwareMap.get(DcMotorEx.class, "armDown");
        armUp.setMode(RUN_USING_ENCODER);
        armDown.setMode(RUN_USING_ENCODER);
        armUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armUp.setDirection(DcMotorSimple.Direction.REVERSE);
        armDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDown.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            armPowerDown = Math.max(armMinPower, Math.min(coefficient*Math.cos(armUp.getCurrentPosition()*Math.PI/2000),1));
            if(gamepad1.left_stick_y > 0){
                if(armUp.getCurrentPosition() < SSValues.ARM_UP){
                    armUp.setPower(gamepad1.left_stick_y*armPowerUp);
                    armDown.setPower(gamepad1.left_stick_y*armPowerUp);
                }else{
                    armUp.setPower(0);
                    armDown.setPower(0);
                }
            }else if(gamepad1.left_stick_y < 0){
                armUp.setPower(gamepad1.left_stick_y*armPowerDown);
                armDown.setPower(gamepad1.left_stick_y*armPowerDown);
            }
            else{
                armUp.setPower(0);
                armDown.setPower(0);
            }

            telemetry_M.addData("ArmUp Power", armUp.getPower());
            telemetry_M.addData("ArmDown Power", armDown.getPower());
            telemetry_M.addData("ArmUp Encoder", armUp.getCurrentPosition());
            telemetry_M.addData("ArmDown Encoder", armDown.getCurrentPosition());
            telemetry_M.update();

        }
    }
}
