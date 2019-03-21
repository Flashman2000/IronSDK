package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class VarRepo {

    public DcMotor RF;
    public DcMotor LF;
    public DcMotor RB;
    public DcMotor LB;

    public DcMotor linAct;

    public WebcamName webcam;

    public float RFPwr;
    public float LFPwr;
    public float RBPwr;
    public float LBPwr;
    public float linActPwr;

    public float channel1;
    public float channel2;
    public float channel3;
    public float goalHeading;


    public GoldAlignDetector autoAlignDetector;

    boolean aligned;
    double pos;
    int count;
    boolean left;
    boolean right;
    boolean center;

    String mineralLoc;

    BNO055IMU imu;

    public Orientation angles;
    float heading;


}
