/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Ironclad;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the RobotConfigs Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled RobotConfigs
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Home Base Red")
public class AutoHomeBaseRed extends LinearOpMode
{

    RobotConfigs robot = new RobotConfigs();
    RobotMovements roboAction = new RobotMovements();

    private boolean center = false;
    private boolean left = false;
    private boolean right = false;
    public Orientation angles;
    float goalHeading = -80;
    double goalPitch = -89;
    public float pitch;
    float heading;
    boolean aligned;
    double pos;
    int count = 0;

    @Override
    public void runOpMode() {

        robot.initAuto(hardwareMap, telemetry);

        waitForStart();
        telemetry.clear();

        while (opModeIsActive()) {

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            pitch = angles.thirdAngle;

            while ((opModeIsActive() && count < 2)) {
                telemetry.addData("IsAligned", robot.detector.getAligned()); // Is the RobotConfigs aligned with the gold mineral
                telemetry.addData("X Pos", robot.detector.getXPosition()); // Gold X pos.
                telemetry.update();
                aligned = robot.detector.getAligned();
                pos = robot.detector.getXPosition();
                sleep(500);
                count++;
            }

            telemetry.addData("align_ver", aligned);
            telemetry.addData("pos_ver", pos);
            telemetry.update();

            if (aligned) {
                center = true;
                right = false;
                left = false;
                goalHeading = -80;
                telemetry.addData("Center", center);
                telemetry.update();
            } else if (!aligned) {
                if (pos > 400) {
                    right = true;
                    center = false;
                    left = false;
                    goalHeading = -105;
                    telemetry.addData("Right", right);
                    telemetry.update();
                } else {
                    left = true;
                    center = false;
                    right = false;
                    goalHeading = -55;
                    telemetry.addData("Left", left);
                    telemetry.update();
                }
            }

            sleep(2000);


            robot.linActuator.setPower(1);

            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
            robot.blinkinLedDriver.setPattern(robot.pattern);

            //robot.pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
            //robot.blinkinLedDriver.setPattern(robot.pattern);

            while(robot.touchSensor.getState()){}

            robot.linActuator.setPower(0);

            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
            robot.blinkinLedDriver.setPattern(robot.pattern);

            robot.leftDrive.setPower(0.5);
            robot.rightDrive.setPower(-0.5);

            while (heading > goalHeading && opModeIsActive()) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
            }

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);


            robot.rightDrive.setPower(0.4);
            robot.leftDrive.setPower(0.4);


            sleep(1500);

            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);

            sleep(1000);

            robot.Box.setPower(0);

            robot.rightDrive.setPower(1);
            robot.leftDrive.setPower(1);

            if (center) {
                sleep(550);
            }if (right) {
                sleep(250);
            }else{
                sleep(300);
            }


                robot.rightDrive.setPower(0);
                robot.leftDrive.setPower(0);


                sleep(800);

                robot.boxWinch.setPower(0);

                if (left) {
                    robot.leftDrive.setPower(-0.5);
                    robot.rightDrive.setPower(0.5);
                } else {
                    robot.leftDrive.setPower(0.5);
                    robot.rightDrive.setPower(-0.5);
                }


                if (right) {
                    sleep(1800);
                } else if (center) {
                    sleep(2600);
                } else if (left) {
                    sleep(1800);
                }


                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

                sleep(200);

                if(left || right){
                    robot.leftDrive.setPower(-1);
                    robot.rightDrive.setPower(-1);

                    sleep(1000);

                    robot.leftDrive.setPower(0);
                    robot.rightDrive.setPower(0);
                }

                if(left){
                    robot.rightDrive.setPower(1);
                    robot.leftDrive.setPower(-1);
                    sleep(250);
                    robot.leftDrive.setPower(0);
                    robot.rightDrive.setPower(0);
                }

                robot.release.setPosition(1)
                ;

                sleep(2000);

                robot.release.setPosition(0);


                /*
                robot.rightDrive.setPower(-0.35);
                robot.leftDrive.setPower(0.35);

                sleep(820);

                robot.rightDrive.setPower(0.6);
                robot.leftDrive.setPower(0.6);

                while(robot.rangeBack.getDistance(DistanceUnit.MM) < 2000){}

                while (robot.rangeFront.getDistance((DistanceUnit.MM)) > 1150){}

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0.75);

                sleep(300);

                robot.leftDrive.setPower(0);
                robot.leftDrive.setPower(0);
                */

                break;

            }

        }
}
