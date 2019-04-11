package org.firstinspires.ftc.teamcode.Ironclad;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.TransitionSoftware.AutoTransitioner;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {

    RobotConfigs robot = new RobotConfigs();
    RobotMovements uselessLULW = new RobotMovements();


    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry);

        AutoTransitioner.transitionOnStop(this, "Drive");

        waitForStart();

        telemetry.clear();


        String worientation = robot.scan(this, telemetry);

        telemetry.addLine(worientation);

        if(worientation == "C"){

            robot.claim.setPosition(0.5);

        }

        if(worientation == "L"){

            robot.claim.setPosition(0);

        }

        if(worientation == "R"){

            robot.claim.setPosition(1);

        }


        telemetry.addData("Servo Pos", robot.claim.getPosition());
        telemetry.update();

        sleep(2000);

    }


}
