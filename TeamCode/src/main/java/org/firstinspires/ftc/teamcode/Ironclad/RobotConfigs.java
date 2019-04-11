package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;



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

        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        piv1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piv2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piv2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        piv2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linAct.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RB.setDirection(DcMotorSimple.Direction.REVERSE);

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

        autoAlignDetector = new GoldAlignDetector();
        autoAlignDetector.VUFORIA_KEY = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09";
        autoAlignDetector.init(hwm.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM,
                false, webcam);
        autoAlignDetector.useDefaults();
        autoAlignDetector.enable();

    }

    public void initTele(HardwareMap hwm, Telemetry tel)
    {

        RB = hwm.get(DcMotor.class, "RB");
        LB = hwm.get(DcMotor.class, "LB");

        piv1 = hwm.get(DcMotor.class, "piv1");
        piv2 = hwm.get(DcMotor.class, "piv2");

        linAct = hwm.get(DcMotor.class, "linAct");

        spool = hwm.get(DcMotor.class, "spool");

        collec = hwm.get(DcMotor.class, "coll");

        box = hwm.get(Servo.class, "box");

        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        piv1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piv2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piv2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        piv2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linAct.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void RCActivity(Gamepad gp1, Gamepad gp2, Telemetry tel, LinearOpMode opmode){

        channel1 = -gp1.left_stick_y;
        channel3 =  gp1.right_stick_x;

        if(gp1.a){

            box.setPosition(1);

        }

        if(gp1.y){

            box.setPosition(0);

        }

        RBPwr = Range.clip(channel1 - channel3, -1, 1);
        LBPwr = Range.clip(channel1 + channel3, -1, 1);

        linActPwr = Range.clip(gp1.right_trigger - gp1.left_trigger, -1, 1);

        piv1Pwr = Range.clip(-gp2.right_stick_y, -1, 1);
        piv2Pwr = Range.clip(gp2.right_stick_y, -1, 1);

        spoolPwr = Range.clip(gp2.left_stick_y, -1, 1);

        RB.setPower(-RBPwr*0.75);
        LB.setPower(-LBPwr*0.75);


        linAct.setPower(linActPwr);

        piv1.setPower(piv1Pwr*0.5);
        piv2.setPower(piv2Pwr*0.5);

        spool.setPower(spoolPwr);
        collec.setPower(gp2.right_trigger - gp2.left_trigger);



        tel.addData("Pivot Pos", piv2.getCurrentPosition());
        tel.update();

        if(gp2.a){

            box.setPosition(0);

        }

        if(gp2.y){

            piv2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            piv2.setTargetPosition(900);
            if (piv2.getCurrentPosition() < 900){
                piv2.setPower(0.6);
                piv1.setPower(-0.6);
            }else{
                piv2.setPower(-0.6);
                piv1.setPower(0.6);
            }

            box.setPosition(0.3);

            while ((opmode.opModeIsActive() && piv2.isBusy())){

                channel1 = -gp1.left_stick_y;
                channel3 =  gp1.right_stick_x;

                RBPwr = Range.clip(channel1 - channel3, -1, 1);
                LBPwr = Range.clip(channel1 + channel3, -1, 1);

                linActPwr = Range.clip(gp1.right_trigger - gp1.left_trigger, -1, 1);

                piv1Pwr = Range.clip(gp2.left_stick_y, -1, 1);
                piv2Pwr = Range.clip(-gp2.left_stick_y, -1, 1);

                spoolPwr = Range.clip(gp2.right_stick_y, -1, 1);

                RB.setPower(-RBPwr*0.75);
                LB.setPower(-LBPwr*0.75);

                collec.setPower(gp2.right_trigger-gp2.left_trigger);

                linAct.setPower(linActPwr);

                //piv1.setPower(piv1Pwr);
                //piv2.setPower(piv2Pwr);

                spool.setPower(spoolPwr);

                tel.addData("Pivot Pos", piv2.getCurrentPosition());
                tel.update();

            }
            piv2.setPower(0);
            piv1.setPower(0);
            piv2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        if(gp2.x){

            piv2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            piv2.setTargetPosition(1900);
            piv2.setPower(0.6);
            piv1.setPower(-0.6);
            box.setPosition(0.1);

            while ((opmode.opModeIsActive() && piv2.isBusy())){

                channel1 = -gp1.left_stick_y;
                channel3 =  gp1.right_stick_x;

                RBPwr = Range.clip(channel1 - channel3, -1, 1);
                LBPwr = Range.clip(channel1 + channel3, -1, 1);

                linActPwr = Range.clip(gp1.right_trigger - gp1.left_trigger, -1, 1);

                piv1Pwr = Range.clip(gp2.left_stick_y, -1, 1);
                piv2Pwr = Range.clip(-gp2.left_stick_y, -1, 1);

                spoolPwr = Range.clip(gp2.right_stick_y, -1, 1);

                RB.setPower(-RBPwr*0.75);
                LB.setPower(-LBPwr*0.75);

                collec.setPower(gp2.right_trigger-gp2.left_trigger);

                linAct.setPower(linActPwr);

                //piv1.setPower(piv1Pwr);
                //piv2.setPower(piv2Pwr);

                spool.setPower(spoolPwr);

                tel.addData("Pivot Pos", piv2.getCurrentPosition());
                tel.update();

            }
            piv2.setPower(0);
            piv1.setPower(0);
            piv2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

}