package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
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

//import static org.firstinspires.ftc.teamcode.Ironclad.IronAutonomous_BETA.COUNTS_PER_INCH;

public class robot {

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor horzSlide = null;
    public DcMotor vertSlide = null;
    public DcMotor linActuator = null;
    public DcMotor boxWinch = null;

    public Servo release = null;
    double serv = 0;

    public CRServo Box = null;

    public BNO055IMU imu;

    public Orientation angles;
    public Acceleration gravity;

    public WebcamName webcamName;
    
    public GoldAlignDetector detector = new GoldAlignDetector();
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    //WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    HardwareMap hwm = null;
    Telemetry tele = null;
    private ElapsedTime period = new ElapsedTime();

    public robot(){}

    public void initAuto(HardwareMap ahwm, Telemetry tel){

        hwm = ahwm;
        tele = tel;

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

        imu.initialize(parameters);

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        horzSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        vertSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linActuator.setDirection(DcMotorSimple.Direction.FORWARD);



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

        //allTrackables.addAll(targetsRoverRuckus);


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
    public void initTele(HardwareMap ahwm, Telemetry tel){
        hwm = ahwm;
        tele = tel;

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

        imu.initialize(parameters);

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        horzSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        vertSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linActuator.setDirection(DcMotorSimple.Direction.FORWARD);



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

    }

    public void startRcActivity(Gamepad gp1, Gamepad gp2, Telemetry tel){

        tele = tel;

        double left;
        double right;
        double horz = 0;
        double vert = 0;
        double lin;
        double winch;


        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        left = -gp1.left_stick_y;
        right = -gp1.right_stick_y;
        lin = gp1.right_trigger - gp1.left_trigger;
        winch = (-gp2.left_stick_y)/3;

        if(gp1.a){
            serv = 1;
        }else if(gp1.b){
            serv = -1;
        }else if(gp1.y){
            serv = 0;
        }

        if(gp2.a){
            release.setPosition(1);
        }else if(gp2.b){
            release.setPosition(0);
        }

        if(gp2.dpad_right){
            horz = 1;
        }else if(gp2.dpad_left){
            horz = -1;
        }else{
            horz = 0;
        }

        if(gp2.dpad_up){
            vert = 1;
        }else if(gp2.dpad_down){
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