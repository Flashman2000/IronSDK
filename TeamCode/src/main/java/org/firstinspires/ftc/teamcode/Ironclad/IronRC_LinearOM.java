package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "actual drive")
public class IronRC_LinearOM extends LinearOpMode{

    RobotConfigs robot = new RobotConfigs();


    @Override
    public void runOpMode(){

        robot.initTele(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()){

            robot.startRcActivityLinOM(gamepad1, gamepad2, telemetry, this);
            //robot.composetelemetry(telemetry);
            telemetry.update();

        }

    }
}
