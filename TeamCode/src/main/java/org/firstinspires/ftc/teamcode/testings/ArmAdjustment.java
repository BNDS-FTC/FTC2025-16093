package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;

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
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            armPowerDown = Math.max(armMinPower, Math.min(coefficient*Math.cos(arm.getCurrentPosition()*Math.PI/2000),1));
            if(gamepad1.left_stick_y > 0){
                if(arm.getCurrentPosition() < SSValues.ARM_UP){
                    arm.setPower(gamepad1.left_stick_y*armPowerUp);
                }else{
                    arm.setPower(0);
                }
            }else if(gamepad1.left_stick_y < 0){
                arm.setPower(gamepad1.left_stick_y*armPowerDown);
            }
            else{
                arm.setPower(0);
            }

            telemetry_M.addData("Arm Power", arm.getPower());
            telemetry_M.update();

        }
    }
}
