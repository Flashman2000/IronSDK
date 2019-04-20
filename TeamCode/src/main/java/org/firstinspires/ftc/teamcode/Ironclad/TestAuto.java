package org.firstinspires.ftc.teamcode.Ironclad;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Auto")
@Disabled
public class TestAuto extends LinearOpMode {

    RobotConfigs robot = new RobotConfigs();

    @Override
    public void runOpMode(){
        robot.initAuto(hardwareMap, telemetry);
        //robot.autoAlignDetector.disable();
        waitForStart();
        robot.creepWithDistance(this, 0.2, 20, telemetry);
        sleep(2000);

    }
}
