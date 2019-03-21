package org.firstinspires.ftc.teamcode.Ironclad;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class RobotMovements extends VarRepo {

    RobotConfigs robot = new RobotConfigs();

    public void stop() {
        RB.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        LF.setPower(0);
    }

    public void move(double power) {

        RB.setPower(power);
        RF.setPower(power);
        LB.setPower(power);
        LF.setPower(power);

    }

    public void move(double power, long time, LinearOpMode opmode) {

        move(power);

        opmode.sleep(time);

        stop();
    }

    public void turnLeft(double power){

        RB.setPower(power);
        RF.setPower(power);
        LF.setPower(-power);
        LB.setPower(-power);

    }

    public void turnLeft(double power, long time, LinearOpMode opmode){

        turnLeft(power);
        opmode.sleep(time);
        stop();

    }

    public void turnRight(double power){

        RB.setPower(-power);
        RF.setPower(-power);
        LF.setPower(power);
        LB.setPower(power);

    }

    public void turnRight(double power, long time, LinearOpMode opmode){

        turnRight(power);

        opmode.sleep(time);

        stop();

    }

    public void strafeLeft(double power) {

        LF.setPower(-power);
        LB.setPower(power);
        RF.setPower(power);
        RB.setPower(-power);

    }

    public void strafeLeft(double power, long time, LinearOpMode opmode) {

        strafeLeft(power);

        opmode.sleep(time);

        stop();

    }

    public void strafeRight(double power) {

        LF.setPower(power);
        LB.setPower(-power);
        RF.setPower(-power);
        RB.setPower(power);

    }

    public void strafeRight(double power, long time, LinearOpMode opmode) {

        strafeRight(power);

        opmode.sleep(time);

        stop();

    }

    public void deploy(double power, long time, LinearOpMode opmode) {

        linAct.setPower(power);

        opmode.sleep(time);

        linAct.setPower(0);

    }

    public String scan(LinearOpMode opmode, Telemetry tel) {

        while (opmode.opModeIsActive() && count < 2) {

            tel.addData("IsAligned", robot.autoAlignDetector.getAligned());
            tel.addData("X Pos", robot.autoAlignDetector.getXPosition());
            tel.update();
            aligned = robot.autoAlignDetector.getAligned();
            pos = robot.autoAlignDetector.getXPosition();
            opmode.sleep(500);
            count++;

        }

        tel.addData("align_ver", aligned);
        tel.addData("pos_ver", pos);
        tel.update();

        if (aligned) {
            center = true;
            right = false;
            left = false;
            goalHeading = 95;
            tel.addData("Center", center);
            tel.update();
            return "C";
        }
        if (!aligned && pos > 400) {
            right = true;
            center = false;
            left = false;
            goalHeading = 55;
            tel.addData("Right", right);
            tel.update();
            return "R";
        }
        else {
            left = true;
            center = false;
            right = false;
            goalHeading = 135;
            tel.addData("Left", left);
            tel.update();
            return "L";
        }

    }

    public void delpoyAndKnock(LinearOpMode opmode, Telemetry tel, double power){

        mineralLoc = scan(opmode, tel);

        opmode.sleep(500);

        deploy(-1, 3000, opmode);

        if(mineralLoc == "C"){

            turnLeft(power);

            while(opmode.opModeIsActive() && heading < 50){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            stop();

        }

        if (mineralLoc == "R") {

            turnLeft(power);

            while (opmode.opModeIsActive() && heading < 20){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;

            }

            stop();

        }

        if (mineralLoc == "L") {

            turnLeft(power);

            while(opmode.opModeIsActive() && heading < 180){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;

            }

            stop();

        }

    }








}

