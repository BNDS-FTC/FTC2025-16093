package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Disable Servo Test")
@Config
public class DisableServoTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public String servo_name1 = "intakeLeft";
    private Servo servo0=null;
    public String servo_name2 = "intakeRight";
    private Servo servo2=null;
    private LynxModule a;
    @Override
    public void runOpMode() {

        servo0 = hardwareMap.get(Servo.class, servo_name1);
        servo2 = hardwareMap.get(Servo.class, servo_name2);
        int ServoPort=servo0.getPortNumber();
        ServoController Controller=servo0.getController();
        waitForStart();
        servo0.setPosition(1);
        servo2.setPosition(1);
        while (opModeIsActive()){
            if(gamepad1.a){
                LynxSetServoEnableCommand command = new LynxSetServoEnableCommand(a, ServoPort, false);
                try{
                    command.send();
                } catch (LynxNackException | InterruptedException e) {
                    telemetry_M.addData("error!",e);
                }
            }else if(gamepad1.b){
                Controller.pwmEnable();
            }
        }
    }
}