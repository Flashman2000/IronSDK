package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IronAutonomous_BETA extends LinearOpMode {

    /**
     *
     * Will not be developed until robot lin. slide system is done
     *
     */
    robot robot = new robot();
    public ElapsedTime runtimeauto = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 1;

    @Override
    public void runOpMode(){

        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.linActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.linActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.encoderDrive(DRIVE_SPEED, 7, 5, this, telemetry);

        robot.linActuator.setPower(0);


    }

}
