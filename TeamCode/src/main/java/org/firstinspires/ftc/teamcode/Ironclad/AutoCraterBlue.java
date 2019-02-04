package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Crater Blue")
public class AutoCraterBlue extends LinearOpMode {

    RobotConfigs robot = new RobotConfigs();
    RobotMovements roboAction = new RobotMovements();

    private boolean center = false;
    private boolean left = false;
    private boolean right = false;
    public Orientation angles;
    float goalHeading = -80;
    double goalPitch = -89;
    public float pitch;
    float heading;
    boolean aligned;
    double pos;
    int count = 0;

    @Override
    public void runOpMode() {

        robot.initAuto(hardwareMap, telemetry);

        waitForStart();
        telemetry.clear();

        while (opModeIsActive()) {

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            pitch = angles.thirdAngle;

            while ((opModeIsActive() && count < 2)) {
                telemetry.addData("IsAligned", robot.detector.getAligned()); // Is the RobotConfigs aligned with the gold mineral
                telemetry.addData("X Pos", robot.detector.getXPosition()); // Gold X pos.
                telemetry.update();
                aligned = robot.detector.getAligned();
                pos = robot.detector.getXPosition();
                sleep(500);
                count++;
            }

            telemetry.addData("align_ver", aligned);
            telemetry.addData("pos_ver", pos);
            telemetry.update();

            if (aligned) {
                center = true;
                right = false;
                left = false;
                goalHeading = -80;
                telemetry.addData("Center", center);
                telemetry.update();
            } else if (!aligned) {
                if (pos > 400) {
                    right = true;
                    center = false;
                    left = false;
                    goalHeading = -105;
                    telemetry.addData("Right", right);
                    telemetry.update();
                } else {
                    left = true;
                    center = false;
                    right = false;
                    goalHeading = -55;
                    telemetry.addData("Left", left);
                    telemetry.update();
                }
            }

            sleep(2000);

            robot.linActuator.setPower(0.8);

            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
            robot.blinkinLedDriver.setPattern(robot.pattern);

            //robot.pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
            //robot.blinkinLedDriver.setPattern(robot.pattern);

            while(robot.touchSensor.getState()){}

            robot.linActuator.setPower(0);

            robot.leftDrive.setPower(0.5);
            robot.rightDrive.setPower(-0.5);

            while (heading > goalHeading && opModeIsActive()) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);


            robot.rightDrive.setPower(0.4);
            robot.leftDrive.setPower(0.4);


            sleep(1800);

            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);

            break;

        }
    }
}