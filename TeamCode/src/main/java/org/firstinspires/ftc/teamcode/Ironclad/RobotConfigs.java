package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;



public class RobotConfigs extends VarRepo{

    public RobotConfigs(){}

    public void initAuto(HardwareMap hwm, Telemetry tel){

        RF = hwm.get(DcMotor.class, "RF");
        LF = hwm.get(DcMotor.class, "LF");
        RB = hwm.get(DcMotor.class, "RB");
        LB = hwm.get(DcMotor.class, "LB");

        linAct = hwm.get(DcMotor.class, "linAct");

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
        autoAlignDetector.init(hwm.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM,
                false, webcam);
        autoAlignDetector.useDefaults();
        autoAlignDetector.enable();

    }

    public void initTele(HardwareMap hwm, Telemetry tel){

        RF = hwm.get(DcMotor.class, "RF");
        LF = hwm.get(DcMotor.class, "LF");
        RB = hwm.get(DcMotor.class, "RB");
        LB = hwm.get(DcMotor.class, "LB");

        linAct = hwm.get(DcMotor.class, "linAct");


    }

    public void RCActivity(Gamepad gp1, Gamepad gp2){

        channel1 = -gp1.left_stick_y;
        channel2 =  gp1.left_stick_x;
        channel3 =  gp1.right_stick_x;

        RFPwr = Range.clip(channel1 + channel2 - channel3, -1, 1);
        RBPwr = Range.clip(channel1 - channel2 - channel3, -1, 1);
        LFPwr = Range.clip(channel1 - channel2 + channel3, -1, 1);
        LBPwr = Range.clip(channel1 + channel2 + channel3, -1, 1);

        linActPwr = Range.clip(gp1.right_trigger - gp1.left_trigger, -1, 1);

        RF.setPower(RFPwr);
        RB.setPower(RBPwr);
        LF.setPower(LFPwr);
        LB.setPower(LBPwr);

        linAct.setPower(linActPwr);

    }

}