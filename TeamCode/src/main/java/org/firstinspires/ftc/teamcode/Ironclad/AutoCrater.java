package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Crater-Side Carry :)")
public class AutoCrater extends LinearOpMode {

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

    private ElapsedTime time  = new ElapsedTime();

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
                    goalHeading = -110;
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

            robot.linActuator.setPower(-1);

            sleep(3200);

            //robot.linActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.linActuator.setPower(0);

            robot.leftDrive.setPower(0.4);
            robot.rightDrive.setPower(-0.4);

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

            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);

            if(!left) {
                robot.rightDrive.setTargetPosition(-950);
            }else{
                robot.rightDrive.setTargetPosition(-1050);
            }


            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightDrive.setPower(-0.8);
            robot.leftDrive.setPower(-0.8);

            while ((robot.rightDrive.isBusy())){

            }

            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);

            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(500);

            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(-1);

            while (heading > -160 && opModeIsActive()) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            int tar1 = 0;
            int tar2 = 0;

            if (left){

                tar1 = 925;
                tar2 = 0;
            }
            if(right){

                tar1 = 0;
                tar2 = -800;
            }
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);
            robot.rightDrive.setTargetPosition(-4600+tar1+tar2);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightDrive.setPower(-1);
            robot.leftDrive.setPower(-1);

            while ((robot.rightDrive.isBusy())){

            }

            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);

            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(500);

            robot.leftDrive.setPower(-1);
            robot.rightDrive.setPower(0);

            while (heading < -145 && opModeIsActive()) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);
            robot.rightDrive.setTargetPosition(-3000);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightDrive.setPower(-0.5);
            robot.leftDrive.setPower(-0.5);

            while ((robot.rightDrive.isBusy())){

            }

            robot.leftDrive.setPower(0);

            robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition());

            //robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(700);

            if(center){
                robot.release.setPosition(1);
                robot.leftDrive.setPower(0.5);
                robot.rightDrive.setPower(-0.5);

                while (heading > -180 && opModeIsActive()) {
                    angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = angles.firstAngle;

                    if(heading > 0){
                        break;
                    }
                }

                while(heading < 25 && opModeIsActive()){

                    angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = angles.firstAngle;

                }

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

            }else{
                if (right) {
                    robot.release.setPosition(1);
                    robot.leftDrive.setPower(0.5);
                    robot.rightDrive.setPower(-0.5);

                    while (heading > -180 && opModeIsActive()) {
                        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        heading = angles.firstAngle;

                        if (heading > 0) {
                            break;
                        }
                    }
                    robot.leftDrive.setPower(0);
                    robot.rightDrive.setPower(0);

                }else{

                    telemetry.addLine("Left");
                    telemetry.update();
                    robot.release.setPosition(1);
                    robot.leftDrive.setPower(0.5);
                    robot.rightDrive.setPower(-0.5);

                    while (heading > -180 && opModeIsActive()) {
                        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        heading = angles.firstAngle;

                        if (heading > 0) {


                            break;
                        }
                    }
                }
            }
            robot.release.setPosition(1);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            if(left){

                robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightDrive.setTargetPosition(-400);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightDrive.setPower(-0.5);
                robot.leftDrive.setPower(0.5);

                while (robot.rightDrive.isBusy()){}

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

                robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(right){

                robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightDrive.setTargetPosition(400);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightDrive.setPower(0.5);
                robot.leftDrive.setPower(-0.5);

                while (robot.rightDrive.isBusy()){}

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

                robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }


            sleep(200);

            robot.rightDrive.setPower(0.5);
            robot.leftDrive.setPower(0.5);
            sleep(1700);
            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);
            sleep(250);
            robot.rightDrive.setPower(-1);
            robot.leftDrive.setPower(-1);
            sleep(1000);
            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);

            break;

        }
    }
}