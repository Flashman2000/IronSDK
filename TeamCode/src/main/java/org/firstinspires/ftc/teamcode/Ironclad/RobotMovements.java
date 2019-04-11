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



   /** public void delpoyAndOrient(LinearOpMode opmode, Telemetry tel, double power){

        mineralLoc = scan(opmode, tel);

        opmode.sleep(500);

        deploy(-1, 3000, opmode);

        if(mineralLoc == "C"){

            turnLeft(power);

            while(opmode.opModeIsActive() && heading < 100){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            stop();

        }

        if (mineralLoc == "R") {

            turnLeft(power);

            while (opmode.opModeIsActive() && heading < 50){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;

            }

            stop();

        }

        if (mineralLoc == "L") {

            turnLeft(power);

            while(opmode.opModeIsActive() && heading < 150){

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;

            }

            stop();

        }

    }




    */



}

