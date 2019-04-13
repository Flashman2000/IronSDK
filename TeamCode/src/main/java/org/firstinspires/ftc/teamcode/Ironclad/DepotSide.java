package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TransitionSoftware.AutoTransitioner;

@Autonomous(name="Depot Side")
public class DepotSide extends LinearOpMode {

    RobotConfigs robot = new RobotConfigs();
    private String location;

    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry);

        AutoTransitioner.transitionOnStop(this, "Drive");

        waitForStart();

        telemetry.clear();

        location = robot.scan(this, telemetry);

        robot.delpoyAndOrient(this, telemetry, 0.35);

        if(location == "C"){
            robot.moveWithEncoder(0.3, this, 2600, telemetry);
        }
        if(location == "R"){
            robot.moveWithEncoder(0.3, this, 1600, telemetry);
        }
        if(location == "L"){
            robot.moveWithEncoder(0.3, this, 1800, telemetry);
        }

        if(location == "C"){
            robot.claim.setPosition(1);
            sleep(1000);
            robot.moveWithEncoder(-0.3, this, -2500, telemetry);
        }

    }

}
