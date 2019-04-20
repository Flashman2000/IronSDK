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

        sleep(500);

        if(location == "C"){
            robot.moveWithEncoder(0.3, this, 3000, telemetry);
        }
        if(location == "R"){
            robot.moveWithEncoder(0.3, this, 2100, telemetry);
        }
        if(location == "L"){
            robot.moveWithEncoder(0.3, this, 2300, telemetry);
        }

        if(location == "C"){
            robot.claim.setPosition(1);
            sleep(1000);
            robot.moveWithEncoder(-0.3, this, -2450, telemetry);
            robot.claim.setPosition(0);
        }
        if(location == "L"){
            robot.moveWithEncoder(-0.3, this, -200, telemetry);
            robot.turnRightGyro(0.3, -90, this);
            robot.moveWithEncoder(0.3, this, 1400, telemetry);
            robot.claim.setPosition(1);
            sleep(1000);
            robot.moveWithEncoder(-0.3, this, -1600, telemetry);
            robot.claim.setPosition(0);
        }
        if(location == "R"){

            robot.moveWithEncoder(-0.3, this, -200, telemetry);
            sleep(500);
            robot.turnLeftGyro(0.3, -65, this);
            sleep(500);
            robot.moveWithEncoder(0.3, this, 1200, telemetry);
            robot.claim.setPosition(1);
            sleep(500);
            robot.moveWithEncoder(-0.3, this, -2200, telemetry);
            robot.claim.setPosition(0);

        }

        sleep(500);

        if(location == "C" || location == "L") {
            robot.reOrient(this, telemetry, 0.35, -20);
        }
        if(location == "R"){
            robot.moveWithEncoder(-0.3, this, -1800, telemetry);
            robot.reOrient(this, telemetry, 0.35, -2);
        }

        sleep(500);

        if(location == "C" || location == "L") {
            robot.creepWithDistance(this, 0.3, 35, telemetry);
        }
        if(location == "R"){

            robot.moveWithEncoder(0.5, this, 2800, telemetry);
            robot.turnRightGyro(0.35, -25, this);
            robot.reOrient(this, telemetry, 0.35, -20);
            sleep(500);
            robot.creepWithDistance(this, 0.5, 35, telemetry);

        }

        robot.pivotLeftWithImu(this, telemetry, 0.7, 30);

        sleep(500);

        robot.claim.setPosition(0.66);

        robot.move(0.3);

        while (opModeIsActive()){}

    }

}
