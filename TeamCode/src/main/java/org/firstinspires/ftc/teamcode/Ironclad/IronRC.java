package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Drive")
public class IronRC extends LinearOpMode{

    RobotConfigs robot = new RobotConfigs();

    @Override
    public void runOpMode(){

        robot.initTele(hardwareMap, telemetry);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            robot.RCActivity(gamepad1, gamepad2, telemetry, this);

        }

    }

}
