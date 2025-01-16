package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.references.SSValues;

@TeleOp(name="super structure test")
@Config
public class SuperStructureTest extends LinearOpMode {
    public static int pos =0;
    public static double power =0;
    public static boolean reverse0=false,reverse1=true,read_only=true;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private static DcMotorEx motor0;
    private static DcMotorEx motor1;
    public static String name0="slideLeft",name1="slideRight";
    public static double tolerance = 30;

    @Override
    public void runOpMode() {
        SuperStructure upper = new SuperStructure(this, ()->{},0);
        upper.resetArmEncoder();
        waitForStart();
        motor0 = hardwareMap.get(DcMotorEx.class,name0);
        motor1 = hardwareMap.get(DcMotorEx.class,name1);
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            if(pos!=0){
//                if(Math.abs(upper.getSlidesPosition()-pos)>tolerance){
//                    if(upper.getSlidesPosition()<pos){
//                        if (Math.abs(upper.getSlidesPosition()-pos)>100){
//                            upper.setSlidePower(-power);
//                        }else{
//                            upper.setSlidePower(-power*0.05);
//                        }
//                    }else{
//                        if (Math.abs(upper.getSlidesPosition()-pos)>100){
//                            upper.setSlidePower(power);
//                        }else{
//                            upper.setSlidePower(power*0.05);
//                        }
//                    }
//                }else{
//                    upper.setSlidePower(0);
//                }
                if(Math.abs(pos-motor0.getCurrentPosition())>tolerance){
                    upper.setArmByP(pos,power);
                }else{
                    upper.setArmByP(pos,0);
                }

            }else{
                if(!read_only){
                    motor0.setPower(power);
                    motor1.setPower(power);
                }

            }
            if(reverse0){
                motor0.setDirection(DcMotorSimple.Direction.REVERSE);
            }else{
                motor0.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reverse1){
                motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            }else{
                motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            upper.update();
            telemetry_M.addData("Position",upper.getSlidesPosition());
            telemetry_M.addData("motor0 Position", motor0.getCurrentPosition());
            telemetry_M.addData("motor1 Position", motor1.getCurrentPosition());
            telemetry_M.addData("motor0 Power", motor0.getPower());
            telemetry_M.addData("motor1 Power", motor1.getPower());
//            telemetry_M.addData("Arm power",upper.getArmPower());

            telemetry_M.update();
        }
    }
}
