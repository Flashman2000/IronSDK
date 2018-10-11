package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class robot {

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public String Right = "Right";
    public String Left = "Left";
    public String Center = "Center";
    public String Unknown = "Unknown";

    public SamplingOrderDetector detector;

    HardwareMap hwm = null;
    Telemetry tele = null;
    private ElapsedTime period = new ElapsedTime();

    public robot(){}

    public void init(HardwareMap ahwm, Telemetry tel){

        hwm = ahwm;
        tele = tel;

        leftDrive = hwm.get(DcMotor.class, "leftDrive");
        rightDrive = hwm.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tele.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        detector = new SamplingOrderDetector();
        detector.init(ahwm.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
    }

    public void startRcActivity(Gamepad gp1, Gamepad gp2, Telemetry tel){

        tele = tel;

        double left;
        double right;

        left = -gp1.left_stick_y;
        right = -gp1.right_stick_y;

        leftDrive.setPower(left);
        rightDrive.setPower(right);

        tele.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
        tele.addData("Last Order" , detector.getLastOrder().toString()); // The last known result

        if (detector.getCurrentOrder() == SamplingOrderDetector.GoldLocation.RIGHT){
            tele.addData("Position Tester", Right);
        }else if (detector.getCurrentOrder() == SamplingOrderDetector.GoldLocation.LEFT){
            tele.addData("Position Tester", Left);
        }else if (detector.getCurrentOrder() == SamplingOrderDetector.GoldLocation.CENTER){
            tele.addData("Position Tester", Center);
        }else{
            tele.addData("Position Tester", Unknown);
        }
        tele.update();

    }

}
