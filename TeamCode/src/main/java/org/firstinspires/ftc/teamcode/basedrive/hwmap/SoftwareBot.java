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

package org.firstinspires.ftc.teamcode.basedrive.hwmap;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class SoftwareBot
{
    static final int     COUNTS_PER_MOTOR_REV    = 1120 ;    // Tetrix Motor
    static final int DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final int     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //Re committing to fix weird VCS GITHUB issues
    public static final String     LOGTAG = "ElectroBot";

    static final int targetTolerance = 10;

    public DcMotorEx  fl   = null;
    public DcMotorEx  fr   = null;
    public DcMotorEx  bl   = null;
    public DcMotorEx  br   = null;
    public ColorSensor colorSensor = null;
    private BNO055IMU gyro = null;
    private BNO055IMU.Parameters imuParameters;
    // State used for updating telemetry
    private Orientation lastAngles;
    private double curHeading = 0.0;

    /*public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    */

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime runTime  = new ElapsedTime();

    /* Constructor */
    public SoftwareBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        try {

            Log.v(LOGTAG, "Starting Initialization of robot");
            hwMap = ahwMap;

            // Define and Initialize Motors
            fl = hwMap.get(DcMotorEx.class, "FL");
            fr = hwMap.get(DcMotorEx.class, "FR");
            bl = hwMap.get(DcMotorEx.class, "BL");
            br = hwMap.get(DcMotorEx.class, "BR");

            //colorSensor = hwMap.get(ColorSensor.class, "colorsensor");

            Log.v(LOGTAG, "Initialize motor directions");
            fl.setDirection(DcMotorEx.Direction.FORWARD);
            bl.setDirection(DcMotorEx.Direction.FORWARD);
            fr.setDirection(DcMotorEx.Direction.REVERSE);
            br.setDirection(DcMotorEx.Direction.REVERSE);
            //fl.setTargetPositionTolerance(40);
            //bl.setTargetPositionTolerance(40);
            //fr.setTargetPositionTolerance(40);
            //br.setTargetPositionTolerance(40);


        /*
        Reset all motors speed = 0 and reset encoder ticks. This is our starting position.
         */
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


            Log.i(LOGTAG, "Setting 0 power behaviour");
            fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        }catch (Exception ex){
            Log.e(LOGTAG, "Exception", ex);
        }
    }

    public void initIMU() {
        //Initialize IMU
        gyro = hwMap.get(BNO055IMU.class, "imu");
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        //imuParameters.mode = BNO055IMU.SensorMode.GYRONLY;
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        gyro.initialize(imuParameters);

        lastAngles = new Orientation();
    }

    public void resetDriveMotorEncoders() {
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isGyroCalibrated() {
        Log.i(SoftwareBot.LOGTAG, "imu calib status: " + gyro.getCalibrationStatus().toString());
        return gyro.isGyroCalibrated();
    }

    private void resetHeading() {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        curHeading = 0.0;
    }


    public void turnCounterClockwise(double turnAngle, double power) {
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetHeading();
        error = turnAngle - getHeading();
        while (error >= 1) {
            //Log.i(SoftwareBot.LOGTAG, "Turn Diff: " + error);

            steer = getSteer(error, 0.1);
            leftSpeed = power * steer;
            rightSpeed = -1 * leftSpeed;

            this.setDriveMotorPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
            error = turnAngle - getHeading();
        }
        /*
        this.setDriveMotorPower(power, -1 * power, power, -1 * power);
        while (getHeading() < turnAngle) {
            Log.i(LOGTAG, "Z: " + getHeading());
        }

         */
        this.setDriveMotorPower(0, 0, 0, 0);
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetHeading();
        /*try {
            Thread.sleep(10);
        } catch (Exception ex) {
            Log.e(LOGTAG, "Turn Counter Clockwise error: " + ex.toString());
        }

         */
    }

    public void turnClockwise(double angle, double power) {
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;


        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetHeading();
        error = angle + getHeading();
        while (error >= 1) {
            //Log.i(SoftwareBot.LOGTAG, "Turn Diff: " + error);

            steer = getSteer(error, 0.1);
            rightSpeed = power * steer;
            leftSpeed = -1 * rightSpeed;
            this.setDriveMotorPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
            error = angle + getHeading();
        }
        /*
        this.setDriveMotorPower(-1 * power, power, -1 * power, power);
        while (getHeading() == 0.0) {
            Log.i(LOGTAG, "Z: " + getHeading());
        }
        while (getHeading() > (-1 * angle)) {
            Log.i(LOGTAG, "Z: " + getHeading());
        }

         */
        this.setDriveMotorPower(0, 0, 0, 0);
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetHeading();
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param target Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    /*public double getError(double targetHeading) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetHeading - this.getRobotHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

     */
    public double checkHeading(double target) {
        double correction, angle;

        angle = getRobotHeading() - target;

        if (abs(angle) < 1)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        return correction;
    }


    public double getRobotHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double delatHeading = angles.firstAngle - lastAngles.firstAngle;
        if (delatHeading < -180)
            delatHeading += 360;
        else if (delatHeading > 180)
            delatHeading -= 360;

        curHeading += delatHeading;
        lastAngles = angles;

        // Log.i(SoftwareBot.LOGTAG, "CurHeading: " +curHeading + ":: DELTA: " + delatHeading);
        return curHeading;
    }

    public double getRoll() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.secondAngle);
    }

    public double getPitch() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.thirdAngle);
    }

    public double formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public double formatDegrees(double degrees) {
        return AngleUnit.DEGREES.normalize(degrees);
    }

    public void setDriveMode(DcMotor.RunMode mode) {

        Log.v(LOGTAG, "Setting Motor mode to: " + mode.toString());
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    public void setDriveMotorPower(double flPower, double frPower, double blPower, double brPower) {

        Log.v(LOGTAG, "Setting Motor power : FL = " + flPower + " :FR = " +frPower +
                    " :BL = " +blPower +" :BR = " +brPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        fl.setPower(flPower);
        fr.setPower(frPower);
    }



    //Distance to travel in inches
    public void setTravelDistance(double distance) {
        //Log.v(LOGTAG, "Setting travel distance to : " +distance);
        fl.setTargetPosition(fl.getCurrentPosition()+(int)(distance*COUNTS_PER_INCH));
        fr.setTargetPosition(fr.getCurrentPosition()+(int)(distance*COUNTS_PER_INCH));
        bl.setTargetPosition(bl.getCurrentPosition()+(int)(distance*COUNTS_PER_INCH));
        br.setTargetPosition(br.getCurrentPosition()+(int)(distance*COUNTS_PER_INCH));
    }

    public void logEncoderPosition(){
        Log.d(LOGTAG, "FL Current Position: " +fl.getCurrentPosition());
        Log.d(LOGTAG, "FR Current Position: " +fr.getCurrentPosition());
        Log.d(LOGTAG, "BL Current Position: " +bl.getCurrentPosition());
        Log.d(LOGTAG, "BR Current Position: " +br.getCurrentPosition());
    }

    public void logPIDValues(PIDFCoefficients pidcoeff) {
        Log.v(LOGTAG, "PIDCoeff P: "+ pidcoeff.p);
        Log.v(LOGTAG, "PIDCoeff I: "+ pidcoeff.i);
        Log.v(LOGTAG, "PIDCoeff D: "+ pidcoeff.d);
        Log.v(LOGTAG, "PIDCoeff F: "+ pidcoeff.f);

    }

    public void travelLeft(double distance) {

        fl.setTargetPosition(fl.getCurrentPosition()+(int)(-1*distance*COUNTS_PER_INCH));
        fr.setTargetPosition(fr.getCurrentPosition()+(int)(1*distance*COUNTS_PER_INCH));
        bl.setTargetPosition(bl.getCurrentPosition()+(int)(1*distance*COUNTS_PER_INCH));
        br.setTargetPosition(br.getCurrentPosition()+(int)(-1*distance*COUNTS_PER_INCH));
    }

    public void travelRight(double distance) {

        fl.setTargetPosition(fl.getCurrentPosition()+(int)(1*distance*COUNTS_PER_INCH));
        fr.setTargetPosition(fr.getCurrentPosition()+(int)(-1*distance*COUNTS_PER_INCH));
        bl.setTargetPosition(bl.getCurrentPosition()+(int)(-1*distance*COUNTS_PER_INCH));
        br.setTargetPosition(br.getCurrentPosition()+(int)(1*distance*COUNTS_PER_INCH));
    }


    public boolean isRobotMoving() {
        boolean retVal = false;
        if (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
            retVal = true;

        return retVal;
    }
    public double getMotorSpeed(double currentTicks, double targetTicks, double speed ){


        double ratio;
        double speedCoef;
        double x,y;


        double tickRatio = (Math.abs(currentTicks/targetTicks));

        Log.i(SoftwareBot.LOGTAG, "Curr Tick: " + currentTicks + " target :" + targetTicks + "Ratio :" + tickRatio);
        ratio = Math.abs(currentTicks/targetTicks);
        //ratio =((double)(((int)(((Math.abs(currentTicks/targetTicks))*100)))*0.01));
        //x =(currentTicks/targetTicks);
        x = ((double)((int)(ratio*100))*0.01);
        Log.i(SoftwareBot.LOGTAG, "X: "+x);
        //speedCoef =((double)(((int)((speed * (-1* (Math.pow(((2*x)-1),4)+1)))*100))*0.01));
        y = speed * ((-1 * Math.pow((2*x-1), 8)) + 1);
        speedCoef = ((double)((int)(y*100))*0.01);
        Log.i(ElectroBot.LOGTAG, "ratio and speed coef :" + ratio +"::"+ speedCoef);

        return Range.clip(speedCoef, 0.3, speed) ;

    }

    public int[] getDriveMotorEncoderPositions() {
        int[] dmEncoderticks = new int[4];

        dmEncoderticks[0] = fl.getCurrentPosition();
        dmEncoderticks[1] = fr.getCurrentPosition();
        dmEncoderticks[2] = bl.getCurrentPosition();
        dmEncoderticks[3] = br.getCurrentPosition();

        return dmEncoderticks;
    }

    public void logDriveEncoderPosition() {
        Log.d(LOGTAG, "FL Current Position: " +fl.getCurrentPosition());
        Log.d(LOGTAG, "FR Current Position: " +fr.getCurrentPosition());
        Log.d(LOGTAG, "BL Current Position: " +bl.getCurrentPosition());
        Log.d(LOGTAG, "BR Current Position: " +br.getCurrentPosition());
    }

    public void setPositionPIDF(double p) {
        fl.setPositionPIDFCoefficients(p);
        fr.setPositionPIDFCoefficients(p);
        bl.setPositionPIDFCoefficients(p);
        br.setPositionPIDFCoefficients(p);

    }

    public boolean isYellow(){
        int colorHSV;
        float hue;
         colorHSV = Color.argb(colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue());
         hue = JavaUtil.colorToHue(colorHSV);
        return hue <= 100;
    }



    public void setMotorPIDF(String motor, double p, double i, double d, double f) {
        switch (motor) {
            case "FL":
                fl.setVelocityPIDFCoefficients(p, i, d, f);
                break;
            case "FR":
                fr.setVelocityPIDFCoefficients(p, i, d, f);
                break;
            case "BL":
                bl.setVelocityPIDFCoefficients(p, i, d, f);
                break;
            case "BR":
                br.setVelocityPIDFCoefficients(p, i, d, f);
                break;
            default:
                Log.e(LOGTAG, "Unknown motor code");
        }

    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}








