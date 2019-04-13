package org.firstinspires.ftc.teamcode.Ironclad;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TransitionSoftware.AutoTransitioner;

@Autonomous(name = "Crater Side")
public class CraterSide extends LinearOpMode {

    RobotConfigs robot = new RobotConfigs();
    RobotMovements uselessLULW = new RobotMovements();

    private float heading;
    private String location;

    private Orientation angles;

    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry);

        AutoTransitioner.transitionOnStop(this, "Drive");

        waitForStart();

        telemetry.clear();

        location = robot.scan(this, telemetry);

        robot.delpoyAndOrient(this, telemetry, 0.35);

        if(location == "C") {
            robot.moveWithEncoder(0.3, this, 1300, telemetry);
        }
        if(location == "R"){
            robot.moveWithEncoder(0.3,this, 1300, telemetry);
        }

        if(location == "L"){
            robot.moveWithEncoder(0.3,this, 1500, telemetry);
        }

        sleep(500);

        if(location == "R") {
            robot.moveWithEncoder(-0.3, this, -800, telemetry);
        }
        if(location == "C"){
            robot.moveWithEncoder(-0.3, this, -700, telemetry);
        }
        if(location == "L"){
            robot.moveWithEncoder(-0.3,this, -800, telemetry);
        }
        sleep(500);

        if(location == "C") {
            robot.reOrient(this, telemetry, 0.35, -20);
        }
        if(location == "R"){
            robot.reOrient(this, telemetry, 0.35, -20);
        }
        if(location == "L"){
            robot.reOrient(this, telemetry, 0.35, -20);
        }
        sleep(4500);

        if(location == "C") {
            robot.creepWithDistance(this, 0.2, 35, telemetry);
        }

        if(location == "R"){
            robot.creepWithDistance(this, 0.2, 35, telemetry);
        }
        if(location == "L"){
            robot.creepWithDistance(this, 0.2, 33, telemetry);
        }
        sleep(500);

        robot.pivotLeftWithImu(this, telemetry, 0.7, 30);

        sleep(500);

        robot.moveWithEncoder(1, this, 900, telemetry);

        sleep(500);

        robot.pivotLeftWithImu(this, telemetry, 0.4, 35);

        sleep(500);

        robot.moveWithEncoder(1, this, 1000, telemetry);

        //robot.pivotLeftWithImu(this, telemetry, 0.7, 35);

        robot.claim.setPosition(1);

        sleep(1000);

        //robot.pivotRightWithImu(this, telemetry, 0.7, 35);

        robot.moveWithEncoder(-1, this, -3600, telemetry);

        robot.spool.setPower(-1);
        sleep(1500);
        robot.spool.setPower(0);
    }


}
