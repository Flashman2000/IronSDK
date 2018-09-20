package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="IronDrive_BETA", group="Linear Opmode")
public class IronDrive_BETA extends OpMode {

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
