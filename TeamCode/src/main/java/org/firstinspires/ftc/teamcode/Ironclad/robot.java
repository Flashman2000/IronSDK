package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class robot {

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor horzSlide = null;
    public DcMotor vertSlide = null;
    public DcMotor linActuator = null;

    HardwareMap hwm = null;
    Telemetry tele = null;
    private ElapsedTime period = new ElapsedTime();

    public robot(){}

    public void init(HardwareMap ahwm, Telemetry tel){

        hwm = ahwm;
        tele = tel;

        leftDrive = hwm.get(DcMotor.class, "leftDrive");
        rightDrive = hwm.get(DcMotor.class, "rightDrive");
        horzSlide = hwm.get(DcMotor.class, "horzSlide");
        vertSlide = hwm.get(DcMotor.class, "verSlide");
        linActuator = hwm.get)DcMotor.class, "linAct");

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


    }

    public void startRcActivity(Gamepad gp1, Gamepad gp2, Telemetry tel){

        tele = tel;

        double left;
        double right;
        double horz = 0;
        double vert = 0;
        double lin;

        left = -gp1.left_stick_y;
        right = -gp1.right_stick_y;
        lin = gp1.right_trigger - gp1.left_trigger;

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
            vert = -1
        }else{
            vert = 0;
        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);
        horzSlide.setPower(horz);
        vertSlide.setPower(vert);
        linActuator.setPower(lin);

    }

}
