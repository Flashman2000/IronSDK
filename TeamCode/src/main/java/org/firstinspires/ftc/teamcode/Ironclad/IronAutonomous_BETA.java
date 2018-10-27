package org.firstinspires.ftc.teamcode.Ironclad;

import android.graphics.drawable.GradientDrawable;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name ="bumbaclit")

public class IronAutonomous_BETA extends LinearOpMode {

    robot robot = new robot();
    public ElapsedTime runtimeauto = new ElapsedTime();
    public Orientation angles;
    public float pitch;
    float heading;
    double goalPitch = -90;
    float goalHeading= -80;
    boolean left = false;
    boolean right = false;
    boolean center = false;
    boolean isAligned = false;
    boolean mhm;
    double pos = 0;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private GoldAlignDetector detector;

    @Override
    public void runOpMode(){

        robot.initTele(hardwareMap, telemetry);
        telemetry.addLine("1");
        telemetry.update();
        robot.webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetry.addLine("1;");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        telemetry.addLine("2");
        telemetry.update();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        parameters.fillCameraMonitorViewParent = true;

        parameters.cameraName = robot.webcamName;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();



        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        pitch = angles.thirdAngle;
        heading = angles.firstAngle;

        // Wait for the game to start (driver presses PLAY)
        mhm = detector.getAligned();
        telemetry.addData("ALigned", detector.getAligned());
        telemetry.update();
        if (mhm) {
            isAligned = true;
            center = true;
            goalHeading = -80;
            telemetry.addLine("Center");
            telemetry.update();
        }
        if(!mhm){
            if(detector.getXPosition() > 400){
                isAligned = false;
                right = true;
                goalHeading = -100;
                telemetry.addLine("Right");
                telemetry.update();
            }

            if(detector.getXPosition() < 200){
                isAligned = false;
                left = true;
                goalHeading = -60;
                telemetry.addLine("Left");
                telemetry.update();
            }

        }
        telemetry.addLine("GO NIGGA");
        waitForStart();
        telemetry.clear();



        sleep(2000);


        robot.linActuator.setPower(-1);

        sleep(1000);

        while(pitch < goalPitch && opModeIsActive()){
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            pitch = angles.thirdAngle;
            robot.composetelemetry(telemetry);
            telemetry.update();
        }
        telemetry.clearAll();
        telemetry.update();

        sleep(300);
        robot.linActuator.setPower(0);

        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(-0.5);

        while(heading > goalHeading && opModeIsActive()){
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
            robot.composetelemetry(telemetry);
            telemetry.update();

        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

}
