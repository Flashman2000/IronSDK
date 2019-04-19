package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class RobotConfigs extends VarRepo{

    public RobotConfigs(){}

    public void initAuto(HardwareMap hwm, Telemetry tel){

        RB = hwm.get(DcMotor.class, "RB");
        LB = hwm.get(DcMotor.class, "LB");

        piv1 = hwm.get(DcMotor.class, "piv1");
        piv2 = hwm.get(DcMotor.class, "piv2");

        linAct = hwm.get(DcMotor.class, "linAct");

        spool = hwm.get(DcMotor.class, "spool");

        collec = hwm.get(DcMotor.class, "coll");

        box = hwm.get(Servo.class, "box");

        claim = hwm.get(Servo.class, "claim");

        rangeSensor = hwm.get(ModernRoboticsI2cRangeSensor.class, "range");

        modernRoboticsI2cGyro = hwm.get(ModernRoboticsI2cGyro.class, "mrgyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        piv1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piv2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piv2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        piv2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linAct.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //RB.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwm.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        webcam = hwm.get(WebcamName.class, "Webcam");

        modernRoboticsI2cGyro.calibrate();

        autoAlignDetector = new GoldAlignDetector();
        autoAlignDetector.VUFORIA_KEY = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09";
        autoAlignDetector.init(hwm.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM,
                false, webcam);
        autoAlignDetector.useDefaults();
        autoAlignDetector.setAlignSettings(0, 170);
        autoAlignDetector.enable();
        tel.addLine("Ready to go :)");
        tel.update();

    }

    public void initTele(HardwareMap hwm, Telemetry tel) {

        RB = hwm.get(DcMotor.class, "RB");
        LB = hwm.get(DcMotor.class, "LB");
        piv1 = hwm.get(DcMotor.class, "piv1");
        piv2 = hwm.get(DcMotor.class, "piv2");
        linAct = hwm.get(DcMotor.class, "linAct");
        spool = hwm.get(DcMotor.class, "spool");
        collec = hwm.get(DcMotor.class, "coll");
        slammer = hwm.get(DcMotor.class, "slam");

        box = hwm.get(Servo.class, "box");

        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piv1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piv2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linAct.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collec.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slammer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void RCActivity(Gamepad gp1, Gamepad gp2, Telemetry tel, LinearOpMode opmode){

        channel1 = -gp1.left_stick_y;
        channel3 =  gp1.right_stick_x;

        RBPwr = Range.clip(channel1 - channel3, -1, 1);
        LBPwr = Range.clip(channel1 + channel3, -1, 1);
        linActPwr = Range.clip(gp1.right_trigger - gp1.left_trigger, -1, 1);
        piv1Pwr = Range.clip(-gp2.right_stick_y, -1, 1);
        piv2Pwr = Range.clip(gp2.right_stick_y, -1, 1);
        spoolPwr = Range.clip(gp2.left_stick_y, -1, 1);
        collecPwr = Range.clip(gp2.right_trigger - gp2.left_trigger, -1, 1);

        RB.setPower(-RBPwr*0.75);
        LB.setPower(-LBPwr*0.75);
        linAct.setPower(linActPwr);
        piv1.setPower(piv1Pwr*0.7);
        piv2.setPower(piv2Pwr*0.7);
        spool.setPower(spoolPwr);
        collec.setPower(collecPwr);
        slammer.setPower(0);

        if(gp1.dpad_up){
            slammer.setPower(1);
        }

        if(gp1.dpad_down){
            slammer.setPower(-1);
        }

        if(gp2.dpad_up){
            piv1.setPower(0.375);
            piv2.setPower(-0.375);
        }

        if(gp2.dpad_down){
            piv1.setPower(-0.375);
            piv2.setPower(0.375);
        }

    }

    public String scan(LinearOpMode opmode, Telemetry tel) {

        while (opmode.opModeIsActive() && count < 2) {

            tel.addData("IsAligned", autoAlignDetector.getAligned());
            tel.addData("X Pos", autoAlignDetector.getXPosition());
            tel.update();
            aligned = autoAlignDetector.getAligned();
            pos = autoAlignDetector.getXPosition();
            opmode.sleep(500);
            count++;

        }

        tel.addData("align_ver", aligned);
        tel.addData("pos_ver", pos);
        tel.update();

        if (aligned) {
            center = true;
            right = false;
            left = false;
            goalHeading = 95;
            tel.addData("Center", center);
            tel.update();
            return "C";
        }
        if (!aligned && pos > 400) {
            right = true;
            center = false;
            left = false;
            goalHeading = 55;
            tel.addData("Right", right);
            tel.update();
            return "R";
        }
        else {
            left = true;
            center = false;
            right = false;
            goalHeading = 135;
            tel.addData("Left", left);
            tel.update();
            return "L";
        }

    }

    public void stop() {
        RB.setPower(0);

        LB.setPower(0);

    }

    public void move(double power) {

        RB.setPower(-power);

        LB.setPower(-power);


    }

    public void move(double power, long time, LinearOpMode opmode) {

        move(power);

        opmode.sleep(time);

        stop();
    }

    public void moveWithEncoder(double power, LinearOpMode opmode, int pulses, Telemetry tel){

        resetAngle();

        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(power > 0) {

            while (opmode.opModeIsActive() && LB.getCurrentPosition() > -pulses && RB.getCurrentPosition() > -pulses) {
                LB.setPower(-power);
                RB.setPower(-power);
                tel.addData("RB", RB.getCurrentPosition());
                tel.addData("LB", LB.getCurrentPosition());
                tel.update();

            }
        }else{
            while (opmode.opModeIsActive() && LB.getCurrentPosition() < -pulses && RB.getCurrentPosition() < -pulses) {
                correction = checkDirection(0.01);
                LB.setPower(-power + correction);
                RB.setPower(-power - correction);
                tel.addData("RB", RB.getCurrentPosition());
                tel.addData("LB", LB.getCurrentPosition());
                tel.update();
            }
        }

        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();

    }

    public void turnRight(double power){

        RB.setPower(power);


        LB.setPower(-power);

    }

    public void turnRight(double power, long time, LinearOpMode opmode){

        turnLeft(power);
        opmode.sleep(time);
        stop();

    }

    public void  turnRightGyro(double power, int angle, LinearOpMode opmode){

        turnRight(power);

        while(opmode.opModeIsActive() && heading > angle){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
        }

        stop();

    }

    public void turnLeft(double power){

        RB.setPower(-power);

        LB.setPower(power);

    }

    public void turnLeft(double power, long time, LinearOpMode opmode){

        turnRight(power);

        opmode.sleep(time);

        stop();

    }

    public void turnLeftGyro(double power, int angle, LinearOpMode opmode){

        turnLeft(power);

        while ((opmode.opModeIsActive() && heading < angle)){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
        }

        stop();

    }

    public void strafeLeft(double power) {


        LB.setPower(power);

        RB.setPower(-power);

    }

    public void strafeLeft(double power, long time, LinearOpMode opmode) {

        strafeLeft(power);

        opmode.sleep(time);

        stop();

    }

    public void strafeRight(double power) {


        LB.setPower(-power);

        RB.setPower(power);

    }

    public void strafeRight(double power, long time, LinearOpMode opmode) {

        strafeRight(power);

        opmode.sleep(time);

        stop();

    }

    public void deploy(double power, long time, LinearOpMode opmode) {

        linAct.setPower(power);

        opmode.sleep(time);

        linAct.setPower(0);

    }

    public void delpoyAndOrient(LinearOpMode opmode, Telemetry tel, double power){

        mineralLoc = scan(opmode, tel);

        tel.addData("Location", mineralLoc);
        tel.update();

        autoAlignDetector.disable();

        opmode.sleep(500);

        deploy(-0.5, 5000, opmode);

        if(mineralLoc == "C"){

            turnRight(power);

            while(opmode.opModeIsActive() && heading > -65){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            stop();

        }

        if (mineralLoc == "R") {

            turnRight(power);

            while (opmode.opModeIsActive() && heading > -65){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;

            }
            stop();

            opmode.sleep(500);
            moveWithEncoder(0.3, opmode, 200, tel);

            turnRight(power);

            while (opmode.opModeIsActive() && heading > -95){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            stop();


        }

        if (mineralLoc == "L") {

            turnRight(power);

            while(opmode.opModeIsActive() && heading > -35){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;

            }

            stop();

        }

    }

    public void reOrient(LinearOpMode opmode, Telemetry tel, double power, double angle){

        turnLeft(power);

        while(opmode.opModeIsActive() && heading < angle){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
        }

        stop();

    }

    public void pivotLeftWithImu(LinearOpMode opmode,  Telemetry tel, double power, double angle){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;

        RB.setPower(-power);

        while (opmode.opModeIsActive() && heading < angle) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;

        }

        stop();

    }

    public void pivotRightWithImu(LinearOpMode opmode,  Telemetry tel, double power, double angle){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;

        LB.setPower(-power);


        while (opmode.opModeIsActive() && heading < angle) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;

        }

        stop();

    }

    public void creepWithDistance(LinearOpMode opmode, double power, double distance, Telemetry tel){

        tel.clearAll();

        resetAngle();

        while(opmode.opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > distance) {
            correction = checkDirection(0.01);
            LB.setPower(-power + correction);
            RB.setPower(-power - correction);
            tel.addData("Target Dis", distance);
            tel.addData("Current Dis", rangeSensor.getDistance(DistanceUnit.CM));
            tel.update();

        }
        tel.clearAll();
        stop();

    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection(double adj)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = adj;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */






}