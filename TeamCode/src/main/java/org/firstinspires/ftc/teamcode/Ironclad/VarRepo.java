package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class VarRepo {

    public DcMotor RF;
    public DcMotor LF;
    public DcMotor RB;
    public DcMotor LB;

    public WebcamName webcam;

    public float RFPwr;
    public float LFPwr;
    public float RBPwr;
    public float LBPwr;

    public float channel1;
    public float channel2;
    public float channel3;

    public GoldAlignDetector autoAlignDetector;



}
