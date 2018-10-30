package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class RobotMovements extends VarRepo {

    RobotConfigs robot = new RobotConfigs();

    public void RobotMovements(){
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        pitch = angles.thirdAngle;
    }

    public void MineralScan(LinearOpMode opmode, Telemetry tel){

        while (opmode.opModeIsActive() && count < 2){
            tel.addData("IsAligned", robot.detector.getAligned());
            tel.addData("X Pos", robot.detector.getXPosition());
            tel.update();
            aligned = robot.detector.getAligned();
            pos = robot.detector.getXPosition();
            opmode.sleep(500);
            count++;
        }

    }

    public void MineralPickup(Telemetry tel){

        if(aligned){
            center = true; right = false; left = false;
            goalHeading = -80;
            tel.addData("Center", center);
            tel.update();
        }else if(!aligned){
            if(robot.detector.getXPosition() > 400){
                right = true; left = false; center = false;
                goalHeading = -105;
                tel.addData("Right", right);
                tel.update();
            }else{
                left = true; center = false; right = false;
                goalHeading = -55;
                tel.addData("Left", left);
                tel.update();
            }
        }

    }

    public void descend (double power){
        robot.linActuator.setPower(power);
    }

    public void stopLinearActuator(){
        robot.linActuator.setPower(0);
    }

    public void flatten(double goalPitch, LinearOpMode opmode){

        while(opmode.opModeIsActive() && pitch < goalPitch){
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            pitch = angles.thirdAngle;
        }

        opmode.sleep(300);
        robot.linActuator.setPower(0);

    }

    public void turnToAngle(LinearOpMode opmode) {

        while (opmode.opModeIsActive() && heading > goalHeading){
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
        }
    }

    public void turn(double rightPwr, double leftPwr){
        robot.rightDrive.setPower(rightPwr);
        robot.leftDrive.setPower(leftPwr);
    }

    public void drive(double pwr){
        robot.rightDrive.setPower(pwr);
        robot.leftDrive.setPower(pwr);
    }

    public void stop(){
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public void CollectMineralAndDeployRQ(LinearOpMode opmode){
        robot.boxWinch.setPower(-0.6);

        opmode.sleep(200);

        robot.boxWinch.setPower(0);

        drive(0.4);

        robot.Box.setPower(-1);

        opmode.sleep(1500);

        stop();

        opmode.sleep(1000);

        robot.Box.setPower(0);

        drive(1);

        opmode.sleep(100);

        stop();

        opmode.sleep(200);

        turn(-0.5, 0.5);

        opmode.sleep(2000);

        stop();

        robot.release.setPosition(1);

    }



}
