package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Auto_With_Int extends LinearOpMode {

    RobotMovements Shookree = new RobotMovements();
    RobotConfigs Configuration = new RobotConfigs();


    @Override
    public void runOpMode(){

        Configuration.initAuto(hardwareMap, telemetry);

        waitForStart();
        telemetry.clear();

        while (opModeIsActive()){

            Shookree.MineralScan(this, telemetry);

            Shookree.MineralPickup(telemetry);

            sleep(1000);

            Shookree.descend(1);

            Shookree.flatten(-89, this);

            sleep(300);

            Shookree.stopLinearActuator();

            Shookree.turn(-0.5, 0.5);

            Shookree.turnToAngle(this);

            Shookree.stop();

            Shookree.CollectMineralAndDeployRQ(this);

        }


    }

}
