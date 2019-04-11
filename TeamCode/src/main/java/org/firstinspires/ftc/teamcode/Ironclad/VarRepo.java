package org.firstinspires.ftc.teamcode.Ironclad;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class VarRepo {

    public DcMotor RF;
    public DcMotor LF;
    public DcMotor RB;
    public DcMotor LB;

    public DcMotor linAct;

    public DcMotor piv1;
    public DcMotor piv2;

    public DcMotor spool;

    public Servo claim;

    public Servo box;

    public DcMotor collec;

    public WebcamName webcam;

    public float RFPwr;
    public float LFPwr;
    public float RBPwr;
    public float LBPwr;

    public float piv1Pwr;
    public float piv2Pwr;

    public float linActPwr;

    public float spoolPwr;

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

    boolean collectOn = false;


}
