package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@Config
public class TestSlidePID extends LinearOpMode {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this);
        NewMecanumDrive drive =new NewMecanumDrive( );
        //XCYBoolean testMove = new XCYBoolean(()->gamepad1.b);
        superstructure.resetSlide();

        waitForStart();

        superstructure.setSlidePosition(0);
        superstructure.setArmPosition(0);
        Runnable update = ()->{drive.update();superstructure.update();XCYBoolean.bulkRead();};

        while (opModeIsActive()) {
            if (gamepad1.a) {
                superstructure.setSlidePosition(SSValues.SLIDE_MIN);
            }
            if(gamepad1.y) {
                superstructure.setSlidePosition(SSValues.SLIDE_MAX);
            }
            if(gamepad1.x) {
                superstructure.setSlidePosition(SSValues.SLIDE_MIDDLE);
            }

            telemetry_M.addData("arm:", superstructure.getArmPosition());
            telemetry.addData("slideL: ",superstructure.getSlideLeftPosition());
            telemetry.addData("slideR: ",superstructure.getSlideRightPosition());
            telemetry_M.addData("Slide Error",superstructure.getSlidePosition() - superstructure.getSlideTargetPosition());
            telemetry_M.update();
            update.run();
        }
    }

}
