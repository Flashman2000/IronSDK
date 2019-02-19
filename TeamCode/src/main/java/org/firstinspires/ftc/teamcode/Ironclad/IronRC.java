package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot Control")
@Disabled
public class IronRC extends OpMode {

    RobotConfigs ironclad = new RobotConfigs();

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

