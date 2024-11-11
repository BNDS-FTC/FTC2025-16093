package org.firstinspires.ftc.teamcode.testings;

import static org.firstinspires.ftc.teamcode.testings.MotorTest.set_power_mode_or_set_position_mode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (group = "Testing")
@Config
public class SlideTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private DcMotorEx mSlideRight = null;
    private double power = 1;
    public static int encoder_position = 1150;
    public static double max_power = 1;
    public static boolean read_only = true;
    public static boolean reverse_left = true;
    public static boolean reverse_right = false;
    public static boolean set_power_mode_or_set_position_mode = false;
    public static boolean reset = true;

    @Override
    public void runOpMode(){
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");

        waitForStart();
        if (reset) {
            mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (reverse_right) {
            mSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        while(opModeIsActive()){
            if (set_power_mode_or_set_position_mode) {
                if (read_only) {
                    mSlideRight.setPower(0);
                }
                else {
                    mSlideRight.setPower(-gamepad1.right_stick_y);
                }
                mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            } else {
                if (!read_only) {
                    mSlideRight.setTargetPosition(encoder_position);
                    mSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mSlideRight.setPower(max_power);

                    sleep(10000);
                }
                telemetry_M.addData("is busy_rightSlide", mSlideRight.isBusy());
            }


            telemetry_M.addData("encoder_slideRight", mSlideRight.getCurrentPosition());

            telemetry_M.addData("right_velocity", mSlideRight.getVelocity());
            telemetry_M.update();
        }
    }
}
