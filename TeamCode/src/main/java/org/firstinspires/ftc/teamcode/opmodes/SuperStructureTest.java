package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp(name = "SuperStructure Test")
public class SuperStructureTest extends LinearOpMode {
    public SuperStructure upper;
    @Override
    public void runOpMode() throws InterruptedException{
        upper = new SuperStructure(this);
        XCYBoolean intakeFar =new XCYBoolean(()->gamepad1.a);
        XCYBoolean intakeNear = new XCYBoolean(()->gamepad1.b);
        XCYBoolean resetPos = new XCYBoolean(()->gamepad1.x);
        XCYBoolean releaseHigh = new XCYBoolean(()->gamepad1.y);

        waitForStart();

        while(opModeIsActive()){
            if(intakeFar.toTrue()){
                upper.intakeFar();
            }
            if(intakeNear.toTrue()){
                upper.intakeNear();
            }
            if(gamepad1.x){
                upper.resetPos();
            }
            if(gamepad1.y){
                upper.releaseHigh();
            }

            upper.update();
            telemetry.addData("arm: ",upper.getArmPosition());
            telemetry.addData("slideL: ",upper.getSlideLeftPosition());
            telemetry.addData("slideR: ",upper.getSlideRightPosition());
            telemetry.update();
            XCYBoolean.bulkRead();
        }
    }
}