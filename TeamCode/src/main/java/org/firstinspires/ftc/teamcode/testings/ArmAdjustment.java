package org.firstinspires.ftc.teamcode.testings;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

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
    public static double armPower;
    public static double armMinPower = 0.35;
    public static double coefficient = 1.4;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx armUp = hardwareMap.get(DcMotorEx.class, "armUp");
        DcMotorEx armDown = hardwareMap.get(DcMotorEx.class, "armDown");
        armUp.setMode(STOP_AND_RESET_ENCODER);
        armDown.setMode(STOP_AND_RESET_ENCODER);
        armUp.setMode(RUN_USING_ENCODER);
        armDown.setMode(RUN_USING_ENCODER);
        armUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armUp.setDirection(DcMotorSimple.Direction.FORWARD);
        armDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armDown.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            armPower = 1;
                    //Math.max(ArmAdjustment.armMinPower, Math.min(ArmAdjustment.coefficient*Math.cos(armUp.getCurrentPosition()*Math.PI/2200),1));

            if(gamepad1.left_stick_y > 0){
                telemetry_M.addData("Arm State", "up");
                armUp.setPower(gamepad1.left_stick_y*armPower);
                armDown.setPower(gamepad1.left_stick_y*armPower);

            }else if(gamepad1.left_stick_y < 0){
                telemetry_M.addData("Arm State", "down");
                armUp.setPower(gamepad1.left_stick_y*armPower);
                armDown.setPower(gamepad1.left_stick_y*armPower);
            }
            else{
                armUp.setPower(0);
                armDown.setPower(0);
                telemetry_M.addData("Arm State", "null");
            }

            telemetry_M.addData("gamepad",gamepad1.left_stick_y);
            telemetry_M.addData("ArmUp Power", armUp.getPower());
            telemetry_M.addData("ArmDown Power", armDown.getPower());
            telemetry_M.addData("ArmUp Encoder", armUp.getCurrentPosition());
            telemetry_M.addData("ArmDown Encoder", armDown.getCurrentPosition());
            telemetry_M.update();

        }
    }
}
