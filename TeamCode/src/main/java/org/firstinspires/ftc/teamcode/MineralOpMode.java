package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.MineralDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DogeCV Mineral Detector", group="DogeCV")

public class MineralOpMode  extends OpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private MineralDetector mineralDetector = null;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");


        mineralDetector = new MineralDetector();
        mineralDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        mineralDetector.areaWeight = 0.02;
        mineralDetector.detectionMode = MineralDetector.MineralDetectionMode.MAX_AREA;

        mineralDetector.debugContours = true;
        mineralDetector.maxDifference = 15;
        mineralDetector.ratioWeight = 15;
        mineralDetector.minArea = 700;

        mineralDetector.enable();
    }

    @Override
    public void init_loop() {telemetry.addData("Status", "Initialized");}

    @Override
    public void start(){runtime.reset();}

    @Override
    public void loop() {

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Current Order", "Mineral Order: " + mineralDetector.getCurrentOrder().toString());
        telemetry.addData("Last Order", "Mineral Order: " + mineralDetector.getLastOrder().toString());


    }

    @Override
    public void stop(){mineralDetector.disable();}
}
