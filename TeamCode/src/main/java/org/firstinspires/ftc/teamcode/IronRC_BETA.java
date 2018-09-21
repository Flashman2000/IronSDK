package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class IronRC_BETA extends OpMode {

    robot ironclad = new robot();

    @Override
    public void init(){ ironclad.init(hardwareMap); }

    @Override
    public void init_loop() {}

    @Override
    public void start(){}

    @Override
    public void loop(){ ironclad.startRcActivity(gamepad1, gamepad2); }

}

