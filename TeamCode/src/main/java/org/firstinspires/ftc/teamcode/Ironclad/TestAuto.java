package org.firstinspires.ftc.teamcode.Ironclad;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.TransitionSoftware.AutoTransitioner;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {

    RobotConfigs config = new RobotConfigs();
    RobotMovements robot = new RobotMovements();


    @Override
    public void runOpMode(){

        config.initAuto(hardwareMap, telemetry);

        AutoTransitioner.transitionOnStop(this, "Drive");

        waitForStart();

        telemetry.clear();

        robot.delpoyAndKnock(this, telemetry, -1);

    }


}
