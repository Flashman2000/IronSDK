package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Ironclad.RobotConfigs;
import org.firstinspires.ftc.teamcode.Ironclad.RobotMovements;

@Autonomous(name="Depot Side")
public class DepotSide extends LinearOpMode {

    RobotConfigs robot = new RobotConfigs();
    RobotMovements roboAction = new RobotMovements();


    private boolean center = false;
    private boolean left = false;
    private boolean right = false;
    public Orientation angles;
    float goalHeading = 80;
    double goalPitch = -89;
    public float pitch;
    float heading;
    boolean aligned;
    double pos;
    int count = 0;

    @Override
    public void runOpMode() {

        robot.initAuto(hardwareMap, telemetry);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        waitForStart();
        telemetry.clear();

        while (opModeIsActive()) {


            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            pitch = angles.firstAngle;

            while (opModeIsActive() && count < 2) {
                telemetry.addData("IsAligned", robot.detector.getAligned());
                telemetry.addData("X Pos", robot.detector.getXPosition());
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
                goalHeading = 95;
                telemetry.addData("Center", center);
                telemetry.update();
            } else if (!aligned) {
                if (pos > 400) {
                    right = true;
                    center = false;
                    left = false;
                    goalHeading = 55;
                    telemetry.addData("Right", right);
                    telemetry.update();
                } else {
                    left = true;
                    center = false;
                    right = false;
                    goalHeading = 135;
                    telemetry.addData("Left", left);
                    telemetry.update();
                }
            }

            sleep(2000);
            robot.linAct.setPower(-1);
            sleep(2950);
            robot.linAct.setPower(0);

            if (!left) {
                robot.leftDrive.setPower(-0.8);
                robot.rightDrive.setPower(0.8);

                while (heading < 35 && opModeIsActive()) {
                    angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = angles.firstAngle;
                }

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
            } else {
                robot.leftDrive.setPower(-0.6);
                robot.rightDrive.setPower(0.6);

                while (heading < 45 && opModeIsActive()) {
                    angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = angles.firstAngle;
                }

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
            }


            robot.rightDrive.setPower(0.5);
            robot.leftDrive.setPower(0.5);
            sleep(250);

            robot.leftDrive.setPower(-1);
            robot.rightDrive.setPower(1);

            while (heading < goalHeading && opModeIsActive()) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            robot.rightDrive.setPower(0.5);
            robot.leftDrive.setPower(0.5);

            if (left) {
                sleep(1100);
            } else {
                sleep(900);
            }
            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);

            sleep(250);

            /*
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightDrive.setTargetPosition(-800);
            robot.leftDrive.setTargetPosition(-800);
            robot.rightDrive.setPower(-0.5);
            robot.leftDrive.setPower(-0.5);

            while (robot.rightDrive.isBusy() && opModeIsActive()) {
            }

            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);*/

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(!right){

                robot.leftDrive.setPower(1);
                robot.rightDrive.setPower(-1);

                while (heading > 90 && opModeIsActive()) {
                    angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = angles.firstAngle;
                }

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

            }else{

                robot.leftDrive.setPower(-1);
                robot.rightDrive.setPower(1);

                while (heading < 90 && opModeIsActive()) {
                    angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = angles.firstAngle;
                }

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
            }

            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.rightDrive.setPower(1);
            robot.leftDrive.setPower(1);
            sleep(500);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.spool.setPower(1);
            sleep(1000);
            robot.spool.setPower(0);

            robot.pivot.setTargetPosition(-4150);

            robot.pivot.setPower(-0.5);

            while(opModeIsActive() && robot.pivot.isBusy()){

            }
            robot.pivot.setPower(0);

            robot.pivot.setTargetPosition(-400);

            robot.pivot.setPower(0.5);

            while(opModeIsActive() && robot.pivot.isBusy()){}

            robot.pivot.setPower(0);


            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



            sleep(200);


            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightDrive.setTargetPosition(-800);
            robot.leftDrive.setTargetPosition(-800);
            robot.rightDrive.setPower(-0.5);
            robot.leftDrive.setPower(-0.5);

            while (robot.rightDrive.isBusy() && opModeIsActive()){
            }

            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);


            break;
        }
    }
}