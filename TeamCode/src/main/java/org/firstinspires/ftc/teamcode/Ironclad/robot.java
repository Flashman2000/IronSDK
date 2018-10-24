package org.firstinspires.ftc.teamcode.Ironclad;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Ironclad.IronAutonomous_BETA.COUNTS_PER_INCH;

public class robot {

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor horzSlide = null;
    public DcMotor vertSlide = null;
    public DcMotor linActuator = null;

    public BNO055IMU imu;

    public Orientation angles;
    public Acceleration gravity;

    HardwareMap hwm = null;
    Telemetry tele = null;
    private ElapsedTime period = new ElapsedTime();

    public robot(){}

    public void init(HardwareMap ahwm, Telemetry tel){

        hwm = ahwm;
        tele = tel;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        leftDrive = hwm.get(DcMotor.class, "leftDrive");
        rightDrive = hwm.get(DcMotor.class, "rightDrive");
        horzSlide = hwm.get(DcMotor.class, "horzSlide");
        vertSlide = hwm.get(DcMotor.class, "verSlide");
        linActuator = hwm.get(DcMotor.class, "linAct");
        imu = hwm.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        composeTelemetry(tel);


    }

    public void startRcActivity(Gamepad gp1, Gamepad gp2, Telemetry tel){

        tele = tel;

        double left;
        double right;
        double horz = 0;
        double vert = 0;
        double lin;

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        tel.update();

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
            vert = -1;
        }else{
            vert = 0;
        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);
        horzSlide.setPower(horz);
        vertSlide.setPower(vert);
        linActuator.setPower(lin);

    }

    void composeTelemetry(Telemetry telemetry) {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void encoderDrive(double speed,
                             double Inches, double timeoutS, LinearOpMode method, Telemetry tel) {
        int newTarget;

        // Ensure that the opmode is still active
        if (method.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = linActuator.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            linActuator.setTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION
            linActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            linActuator.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (method.opModeIsActive() &&
                    (period.seconds() < timeoutS) &&
                    (linActuator.isBusy())) {

                // Display it for the driver.
                tel.addData("Path1",  "Running to %7d :%7d", newTarget);
                tel.addData("Path2",  "Running at %7d :%7d",
                        linActuator.getCurrentPosition());
                tel.update();
            }

            // Stop all motion;
            linActuator.setPower(0);


            // Turn off RUN_TO_POSITION
            linActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

}