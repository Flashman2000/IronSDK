package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DRIVE")

public class IronRC_BETA extends OpMode {

    robot ironclad = new robot();

    @Override
    public void init(){ ironclad.initTele(hardwareMap, telemetry); }

    @Override
    public void init_loop() {}

    @Override
    public void start(){}

    @Override
    public void loop(){ ironclad.startRcActivity(gamepad1, gamepad2, telemetry);
    telemetry.update();
    }

    @Override
    public void stop() {}

}

