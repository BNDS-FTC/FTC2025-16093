package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.Action;


@TeleOp(name = "16093 TeleOp for Experience")
public class TouristTeleOP extends TeleOpMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initTeleOp(()->(upper.alphaAdjustedSampleColor() == 1||upper.alphaAdjustedSampleColor() == 2),-90);

        // Wait until play button is pressed

        waitForStart();


        // Set intake to default stop position and initialize operation mode
//        upper.startIntake();

        // Main control loop while op mode is active
        while (opModeIsActive() && !isStopRequested()) {
            update.run();
            Action.buildSequence(update);
            int TIME_MAX = 60 * 3; //unit: second
            telemetry.addData("已体验时间(min)",(int)(getRuntime()/60+0.5));//四舍五入
            telemetry.update();

            if(getRuntime() >= 0.99 * TIME_MAX &&  getRuntime() <= TIME_MAX) {
                gamepad1.rumble(2000);
            }
            if(getRuntime() >= TIME_MAX) {
                gamepad1.left_stick_button = true;//防止撞坏手腕/滑轨
                sleep(20);  //减速
                upper.sequence = SuperStructure.Sequences.HIGH_CHAMBER_PLACE;//不抖手腕
            }
            if(getRuntime() >= 1.2 * TIME_MAX) {
                break;
            }
        }
    }
}