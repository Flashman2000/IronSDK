package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class robot {

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    HardwareMap hwm = null;
    private ElapsedTime period = new ElapsedTime();

    public robot(){}

    public void init(HardwareMap ahwm){

        hwm = ahwm;

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
    }

    public void startRcActivity(Gamepad gp1, Gamepad gp2){

        double left;
        double right;

        left = -gp1.left_stick_y;
        right = -gp1.right_stick_y;

        leftDrive.setPower(left);
        rightDrive.setPower(right);

    }




}
