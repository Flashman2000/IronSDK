package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

//import static org.firstinspires.ftc.teamcode.Ironclad.AutoHomeBase.COUNTS_PER_INCH;

public class RobotConfigs extends VarRepo{

    public RobotConfigs(){}

    public void initAuto(HardwareMap hwm, Telemetry tel){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        leftDrive = hwm.get(DcMotor.class, "leftDrive");
        rightDrive = hwm.get(DcMotor.class, "rightDrive");
        horzSlide = hwm.get(DcMotor.class, "horzSlide");
        vertSlide = hwm.get(DcMotor.class, "verSlide");
        linActuator = hwm.get(DcMotor.class, "linAct");
        boxWinch = hwm.get(DcMotor.class, "winch");
        Box = hwm.get(CRServo.class, "bx");
        release = hwm.get(Servo.class, "release");
        imu = hwm.get(BNO055IMU.class, "imu");
        webcamName = hwm.get(WebcamName.class, "Webcam 1");
        touchSensor = hwm.get(DigitalChannel.class, "touch");
        rangeBack = hwm.get(DistanceSensor.class, "rback");
        rangeFront = hwm.get(DistanceSensor.class, "rfront");

        Rev2mDistanceSensor disFront = (Rev2mDistanceSensor) rangeFront;
        Rev2mDistanceSensor disBack = (Rev2mDistanceSensor) rangeBack;

        imu.initialize(parameters);

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        horzSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        vertSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linActuator.setDirection(DcMotorSimple.Direction.FORWARD);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        horzSlide.setPower(0);
        vertSlide.setPower(0);
        linActuator.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horzSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horzSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        composetelemetry(tel);
        tel.addLine("1");
        tel.update();

        webcamName = hwm.get(WebcamName.class, "Webcam 1");

        tel.addLine("1;");
        tel.update();

        int cameraMonitorViewId = hwm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwm.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        tel.addLine("2");
        tel.update();

        params.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        params.fillCameraMonitorViewParent = true;

        params.cameraName = webcamName;

        vuforia = new Dogeforia(params);
        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of RobotConfigs center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the RobotConfigs's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, params.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hwm.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
        tel.addLine("GO NIGGA");
        tel.update();


    }

    public void initTele(HardwareMap hwm, Telemetry tel){




        leftDrive = hwm.get(DcMotor.class, "leftDrive");
        rightDrive = hwm.get(DcMotor.class, "rightDrive");
        horzSlide = hwm.get(DcMotor.class, "horzSlide");
        vertSlide = hwm.get(DcMotor.class, "verSlide");
        linActuator = hwm.get(DcMotor.class, "linAct");
        boxWinch = hwm.get(DcMotor.class, "winch");
        Box = hwm.get(CRServo.class, "bx");
        release = hwm.get(Servo.class, "release");
        touchSensor = hwm.get(DigitalChannel.class, "touch");
        rangeBack = hwm.get(DistanceSensor.class, "rback");
        rangeFront = hwm.get(DistanceSensor.class, "rfront");

        Rev2mDistanceSensor disFront = (Rev2mDistanceSensor) rangeFront;
        Rev2mDistanceSensor disBack = (Rev2mDistanceSensor) rangeBack;

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        horzSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        vertSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linActuator.setDirection(DcMotorSimple.Direction.FORWARD);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);




        leftDrive.setPower(0);
        rightDrive.setPower(0);
        horzSlide.setPower(0);
        vertSlide.setPower(0);
        linActuator.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horzSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horzSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    public void startRcActivity(Gamepad gp1, Gamepad gp2, Telemetry tel){

        double left;
        double right;
        double horz = 0;
        double vert = 0;
        double lin;
        double winch;

        left = -gp1.left_stick_y;
        right = -gp1.right_stick_y;
        lin = gp1.right_trigger - gp1.left_trigger;
        winch = (-gp2.right_stick_y)/1.7;

        serv = gp2.right_trigger - gp2.left_trigger;

        if(gp1.a){
            release.setPosition(0.2);
        }else if(gp1.b){
            release.setPosition(1);
        }

        if(gp1.y){

            linActuator.setPower(1);

            while(touchSensor.getState()){

                if(!touchSensor.getState()){
                    break;
                }
            }

            linActuator.setPower(0);

        }

        if(gp2.dpad_up){
            horz = 1;
        }else if(gp2.dpad_down){
            horz = -1;
        }else{
            horz = 0;
        }

        if(gp1.dpad_up){
            vert = 1;
        }else if(gp1.dpad_down){
            vert = -1;
        }else{
            vert = 0;
        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);
        horzSlide.setPower(horz);
        vertSlide.setPower(vert);
        linActuator.setPower(lin);
        boxWinch.setPower(winch);
        Box.setPower(serv);
        tel.addData("Pressed", touchSensor.getState());
        tel.addData("Front Distance", rangeFront.getDistance(DistanceUnit.MM));
        tel.addData("Back Distance", rangeBack.getDistance(DistanceUnit.MM));
        tel.update();


    }

    void composetelemetry(Telemetry tel) {

        // At the beginning of each tel update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        tel.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        tel.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        tel.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        tel.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}