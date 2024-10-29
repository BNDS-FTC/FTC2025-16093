package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
public class Whatever extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    SuperStructure superstructure = new SuperStructure(this);
    NewMecanumDrive drive =new NewMecanumDrive( );

    @Override
    public void runOpMode() throws InterruptedException {

        drive.setUp(hardwareMap);
        drive.update();
        superstructure.resetSlide();

        waitForStart();

        while(opModeIsActive()){
            superstructure.setArmPosition(0);

            if(gamepad1.a){
                superstructure.setSlidePosition(SSValues.SLIDE_MIN);
            }
            if(gamepad1.b){
                superstructure.setSlidePosition(SSValues.SLIDE_MAX);
            }

        }


        telemetry_M.addData("SlideL: ",superstructure.getSlideLeftPosition());
        telemetry_M.addData("SlideR: ",superstructure.getSlideLeftPosition());
        telemetry_M.update();

        superstructure.update();
    }
}
