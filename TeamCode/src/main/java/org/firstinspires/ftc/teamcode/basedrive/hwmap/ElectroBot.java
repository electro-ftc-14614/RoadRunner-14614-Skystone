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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.openftc.revextensions2.VexMC29;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * This the main Robot class Electro 14614.
 *
 * ALL MOTORS/SERVOS/SENSORS SHOULD BE DEFINED AND INITIALIZED IN THIS CLASS.
 * THIS CLASS SHOULD BE USED IN ALL OPMODES.
 */
public class ElectroBot {
    public static final String LOGTAG = "ElectroBot" ;
    //Using Android's built-in Logging functionality. Need to define a tag
    //static final String LOGTAG = "ElectroBot";

    //Variables to get encoder ticks to move robot.
    private static final int COUNTS_PER_MOTOR_REV = 560;    // Gobuilda robot
    //private static final int     COUNTS_PER_MOTOR_REV    = 1120 ;    // Tetrix Motor
    private static final int DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final int WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    /* Motors. servos and sensors */
    private DcMotorEx fl = null;
    private DcMotorEx fr = null;
    private DcMotorEx bl = null;
    private DcMotorEx br = null;
    private DcMotorEx llm = null;
    private DcMotorEx rlm = null;
    private DcMotorEx lim = null;
    private DcMotorEx rim = null;
    private BNO055IMU gyro = null;
    private Servo wgr = null;
    private Servo wgl = null;
    private Servo gripper = null;
    private Servo turner = null;
    private CRServo wml = null;
    private CRServo wmr = null;
    public CRServo aem = null;
    private CRServo tmm = null;
    private Servo fsl = null;
    private Servo fsr = null;
    private Servo css = null;
    private ColorSensor colorSensor1 = null;
    private ColorSensor colorSensor2 = null;

    private HardwareMap hwMap = null;
    private ElapsedTime runTime = new ElapsedTime();
    private Orientation lastAngles;
    private double curHeading = 0.0;

    private float[] hsvValues = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    private final float[] values = hsvValues;
    /* Constructor */
    public ElectroBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        BNO055IMU.Parameters imuParameters;


        // Save reference to Hardware map
        try {

            Log.v(LOGTAG, "Starting Initialization of robot");
            hwMap = ahwMap;

            // Define and Initialize Motors
            fl = hwMap.get(DcMotorEx.class, "FL");
            fr = hwMap.get(DcMotorEx.class, "FR");
            bl = hwMap.get(DcMotorEx.class, "BL");
            br = hwMap.get(DcMotorEx.class, "BR");
            llm = hwMap.get(DcMotorEx.class, "LLM");
            rlm = hwMap.get(DcMotorEx.class, "RLM");
            lim = hwMap.get(DcMotorEx.class, "LIM");
            rim = hwMap.get(DcMotorEx.class, "RIM");
            wgl = hwMap.get(Servo.class, "WGL");
            wgr = hwMap.get(Servo.class, "WGR");
            turner = hwMap.get(Servo.class, "TURNER");
            gripper = hwMap.get(Servo.class, "GRIPPER");
            wml = hwMap.get(CRServo.class, "WML");
            wmr = hwMap.get(CRServo.class, "WMR");
            aem = hwMap.get(CRServo.class, "AEM");
            tmm = hwMap.get(CRServo.class,"TMM");
            fsl = hwMap.get(Servo.class, "FSL");
            fsr = hwMap.get(Servo.class, "FSR");
            css = hwMap.get(Servo.class, "CSS");

            colorSensor1 = hwMap.get(ColorSensor.class, "CS1");
            colorSensor2 = hwMap.get(ColorSensor.class, "CS2");

            //Initialize IMU
            Log.v(LOGTAG, "Initialize IMU");
            gyro = hwMap.get(BNO055IMU.class, "imu");
            // Create new IMU Parameters object.
            imuParameters = new BNO055IMU.Parameters();
            // Use degrees as angle unit.
            //imuParameters.mode = BNO055IMU.SensorMode.GYRONLY;
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            // Express acceleration as m/s^2.
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            // Disable logging.
            imuParameters.loggingEnabled = false;
            // Initialize IMU.
            gyro.initialize(imuParameters);
            gyro.write8(BNO055IMU.Register.OPR_MODE, 0b00000011);

            lastAngles = new Orientation();

            Log.v(LOGTAG, "Initialize motor directions");
            fl.setDirection(DcMotorEx.Direction.FORWARD);
            bl.setDirection(DcMotorEx.Direction.FORWARD);
            fr.setDirection(DcMotorEx.Direction.REVERSE);
            br.setDirection(DcMotorEx.Direction.REVERSE);
            llm.setDirection(DcMotor.Direction.REVERSE);
            rlm.setDirection(DcMotor.Direction.FORWARD);
            lim.setDirection(DcMotor.Direction.FORWARD);
            rim.setDirection(DcMotor.Direction.REVERSE);
            fl.setTargetPositionTolerance(12);
            bl.setTargetPositionTolerance(12);
            fr.setTargetPositionTolerance(12);
            br.setTargetPositionTolerance(12);


            wgl.setDirection(Servo.Direction.FORWARD);
            wgr.setDirection(Servo.Direction.REVERSE);
            gripper.setDirection(Servo.Direction.FORWARD);
            turner.setDirection(Servo.Direction.FORWARD);
            fsl.setDirection(Servo.Direction.FORWARD);
            fsr.setDirection(Servo.Direction.REVERSE);
            css.setDirection(Servo.Direction.FORWARD);


            wml.setDirection(CRServo.Direction.REVERSE);
            wmr.setDirection(CRServo.Direction.FORWARD);
            aem.setDirection(CRServo.Direction.FORWARD);
            tmm.setDirection(CRServo.Direction.FORWARD);
        /*
        Reset all motors speed = 0 and reset encoder ticks. This is our starting position.
         */
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            llm.setPower(0);
            rlm.setPower(0);
            rim.setPower(0);
            lim.setPower(0);

            fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            llm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rlm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rim.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            lim.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


            Log.i(LOGTAG, "Setting 0 power behaviour");
            fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            llm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rlm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lim.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rim.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            Log.i(LOGTAG, "Setting Servo's to Default position");
            this.setWGToUpPosition();
            this.resetTurnerServoPosition();
            this.resetGripperServoPosition();
            this.resetFlipServoPosition();
            this.holdCapstone();

        } catch (Exception ex) {
            Log.e(LOGTAG, "Exception", ex);
        }
    }

    public void resetDriveMotorEncoders() {
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetLiftMotorEncoders() {
        llm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rlm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        llm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rlm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDriveMode(DcMotor.RunMode mode) {

        Log.v(LOGTAG, "Setting Motor mode to: " + mode.toString());
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    public void setDriveMotorPower(double flPower, double frPower, double blPower, double brPower) {

        /*Log.v(LOGTAG, "Setting Motor power : FL = " + flPower + " :FR = " + frPower +
                " :BL = " + blPower + " :BR = " + brPower);*/
        fl.setPower(Range.clip(flPower, -1, 1));
        br.setPower(Range.clip(brPower, -1, 1));
        bl.setPower(Range.clip(blPower, -1, 1));
        fr.setPower(Range.clip(frPower, -1, 1));
    }

    public double getMotorSpeed(double currentTicks, double targetTicks, double speed ){


        double ratio;
        double speedCoef;
        double x,y;


        double tickRatio = (Math.abs(currentTicks/targetTicks));

        Log.i(org.firstinspires.ftc.teamcode.basedrive.hwmap.SoftwareBot.LOGTAG, "Curr Tick: " + currentTicks + " target :" + targetTicks + "Ratio :" + tickRatio);
        ratio = Math.abs(currentTicks/targetTicks);
        //ratio =((double)(((int)(((Math.abs(currentTicks/targetTicks))*100)))*0.01));
        //x =(currentTicks/targetTicks);
        x = ((double)((int)(ratio*100))*0.01);
        Log.i(org.firstinspires.ftc.teamcode.basedrive.hwmap.SoftwareBot.LOGTAG, "X: "+x);
        //speedCoef =((double)(((int)((speed * (-1* (Math.pow(((2*x)-1),4)+1)))*100))*0.01));
        if(ratio<=0.5){
            y = speed * ((-1 * Math.pow((2*x-1), 10)) + 1);
        }
        else{
            y = speed * ((-1 * Math.pow((2*x-1), .4)) + 1);
        }

        /*
        if(tickRatio<.93){
            y = speed * ((-1 * Math.pow((2*x-1), 8)) + 1);
        }
        else if(tickRatio<.98&&tickRatio>.93){
            y=.1;
        }
        else{
            y=0;
           // y = speed * ((1 * Math.pow((x+0.5), -2.6)));

        }
*/
        speedCoef = ((double)((int)(y*100))*0.01);
        Log.i(ElectroBot.LOGTAG, "ratio and speed coef :" + ratio +"::"+ speedCoef);

        return Range.clip(speedCoef, .3, speed) ;

    }
    public void setDriveMotorVelocity(double flVel, double frVel, double blVel, double brVel) {

        Log.v(LOGTAG, "Setting Motor Vel : FL = " + flVel + " :FR = " + frVel +
                " :BL = " + blVel + " :BR = " + brVel);
        bl.setVelocity(blVel);
        br.setVelocity(brVel);
        fl.setVelocity(flVel);
        fr.setVelocity(frVel);
    }

    //public void setmotorpidf();

    public void setLiftMotorPower(double llmPower, double rlmPower) {
        if (llmPower != 0 && rlmPower != 0)
            Log.v(LOGTAG, "LLM: " + llmPower + "; RLM: " + rlmPower);
        llm.setPower(llmPower);
        rlm.setPower(rlmPower);
    }

    public void setIntakeMotorPower(double limPower, double rimPower) {
        lim.setPower(limPower);
        rim.setPower(rimPower);
    }

    //Distance to travel(Forward or Back) in inches
    public void setTravelDistance(double distance) {
        Log.v(LOGTAG, "Setting travel distance to : " + distance);
        fl.setTargetPosition(fl.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH));
        fr.setTargetPosition(fr.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH));
        bl.setTargetPosition(bl.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH));
        br.setTargetPosition(br.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH));
        //Log.v(LOGTAG, "Target Positions: " + fl.getTargetPosition() + ":" + fr.getTargetPosition() + ":" + bl.getTargetPosition() + ":" + br.getTargetPosition());
    }

    public void travelLeft(double distance) {

        fl.setTargetPosition(fl.getCurrentPosition() + (int) (-1 * distance * COUNTS_PER_INCH));
        fr.setTargetPosition(fr.getCurrentPosition() + (int) (1 * distance * COUNTS_PER_INCH));
        bl.setTargetPosition(bl.getCurrentPosition() + (int) (1 * distance * COUNTS_PER_INCH));
        br.setTargetPosition(br.getCurrentPosition() + (int) (-1 * distance * COUNTS_PER_INCH));
    }

    public void travelRight(double distance) {

        fl.setTargetPosition(fl.getCurrentPosition() + (int) (1 * distance * COUNTS_PER_INCH));
        fr.setTargetPosition(fr.getCurrentPosition() + (int) (-1 * distance * COUNTS_PER_INCH));
        bl.setTargetPosition(bl.getCurrentPosition() + (int) (-1 * distance * COUNTS_PER_INCH));
        br.setTargetPosition(br.getCurrentPosition() + (int) (1 * distance * COUNTS_PER_INCH));
    }

    public void setWheelServos(double power) {

        wmr.setPower(power);
        wml.setPower(power);
    }

    public void setArmServos(double power) {
        aem.setPower(power);
    }
    public double getcssposition(){
        double y;
        y=css.getPosition();
        return y;
    }
    public void setTMMPower(double power){
        tmm.setPower(Range.clip(power*1.5,-.9,.9));
    }

    public boolean isRobotMoving() {
        boolean retVal = false;
        if (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
            retVal = true;

        return retVal;
    }

    public void logDriveEncoderPosition() {
        Log.d(LOGTAG, "FL Current Position: " + fl.getCurrentPosition());
        Log.d(LOGTAG, "FR Current Position: " + fr.getCurrentPosition());
        Log.d(LOGTAG, "BL Current Position: " + bl.getCurrentPosition());
        Log.d(LOGTAG, "BR Current Position: " + br.getCurrentPosition());
    }

    public void logPIDValues(PIDFCoefficients pidcoeff) {
        Log.v(LOGTAG, "PIDCoeff P: " + pidcoeff.p);
        Log.v(LOGTAG, "PIDCoeff I: " + pidcoeff.i);
        Log.v(LOGTAG, "PIDCoeff D: " + pidcoeff.d);
        Log.v(LOGTAG, "PIDCoeff F: " + pidcoeff.f);

    }

    public int[] getDriveMotorEncoderPositions() {
        int[] dmEncoderticks = new int[4];

        dmEncoderticks[0] = fl.getCurrentPosition();
        dmEncoderticks[1] = fr.getCurrentPosition();
        dmEncoderticks[2] = bl.getCurrentPosition();
        dmEncoderticks[3] = br.getCurrentPosition();

        return dmEncoderticks;
    }
    public void moveLift(double inches, double speed)
    {
        int position = (int) inches*136;
        //setLiftMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setLiftMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        setLiftMotorEncoderPosition(-position);

        setLiftMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        setLiftMotorPower(-speed, -speed);

    }

    public int[] getLiftMotorEncoderPositions() {
        int[] lmEncoderticks = new int[2];

        lmEncoderticks[0] = llm.getCurrentPosition();
        lmEncoderticks[1] = rlm.getCurrentPosition();

        return lmEncoderticks;
    }

    public void setLiftMotorMode(DcMotor.RunMode mode) {

        Log.v(LOGTAG, "Setting Lift Motor mode to: " + mode.toString());
        llm.setMode(mode);
        rlm.setMode(mode);
    }


    public void setLiftMotorEncoderPosition(int position) {
        this.setLiftMotorEncoderPosition(position, position);
    }

    public void setLiftMotorEncoderPosition(int lmPos, int rmPos) {
//        llm.setTargetPosition(llm.getCurrentPosition() + lmPos);
//        rlm.setTargetPosition(rlm.getCurrentPosition() + rmPos);
        llm.setTargetPosition(lmPos);
        rlm.setTargetPosition(rmPos);
    }

    public void setWGToUpPosition() {
        wgl.setPosition(0.4);
        wgr.setPosition(0.4);
    }

    public void setWGToDownPosition() {
        wgl.setPosition(0.9);
        wgr.setPosition(0.9);
    }

    public void setWGToMidPosition() {
        wgl.setPosition(0.56);
        wgr.setPosition(0.62);
    }

    public void holdCapstone() {
        //Old Vale: 0.15
        //Log.i(ElectroBot.LOGTAG, "HOLDING CAPSTONE");
        css.setPosition(0.85);

    }

    public void dropCapstone() {
        //Old Value 0.85
        //Log.i(ElectroBot.LOGTAG, "Dropping CAPSTONE");
        css.setPosition(0.15);
    }



    public void setTurnerServoPosition() {
        //Old Value: 0.57
        turner.setPosition(0.07);
    }

    public void resetTurnerServoPosition() {
        //Old Value 0.07
        turner.setPosition(0.57);
    }

    public void setGripperServoPosition() {
        gripper.setPosition(0.88);
    }
    public void resetGripperServoPosition() {
        gripper.setPosition(0.5);
    }



    public double[] getCurVelocity() {
        double[] curVel = new double[4];

        curVel[0] = fl.getVelocity();
        curVel[1] = fr.getVelocity();
        curVel[2] = bl.getVelocity();
        curVel[3] = br.getVelocity();

        return curVel;
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

    public void setPositionPIDF(double p) {
        fl.setPositionPIDFCoefficients(p);
        fr.setPositionPIDFCoefficients(p);
        bl.setPositionPIDFCoefficients(p);
        br.setPositionPIDFCoefficients(p);

    }

    public void setFlipServoPosition() {

        fsl.setPosition(.9);
        fsr.setPosition(1);
    }

    public void resetFlipServoPosition() {
        fsl.setPosition(.6);
        fsr.setPosition(.725);

    }

    public double[] getServoPositions() {
        double[] sPos = new double[4];

        sPos[0] = fsl.getPosition();
        sPos[1] = fsr.getPosition();
        sPos[2] = wgl.getPosition();
        sPos[3] = wgr.getPosition();

        return sPos;
    }

    public boolean isYellow(int colorsensor) {
        int colorHSV;
        int R;
        int G;
        int Alpha;

        final double SCALE_FACTOR = 255;



        if (colorsensor == 1) {
            Color.RGBToHSV((int) (colorSensor1.red() * SCALE_FACTOR),
                    (int) (colorSensor1.green() * SCALE_FACTOR),
                    (int) (colorSensor1.blue() * SCALE_FACTOR),
                    hsvValues);

            R = colorSensor1.red();
            G = colorSensor1.green();
            Alpha = colorSensor1.alpha();
        } else {
            Color.RGBToHSV((int) (colorSensor2.red() * SCALE_FACTOR),
                    (int) (colorSensor2.green() * SCALE_FACTOR),
                    (int) (colorSensor2.blue() * SCALE_FACTOR),
                    hsvValues);

            R = colorSensor2.red();
            G = colorSensor2.green();
            Alpha = colorSensor2.alpha();
        }



        Log.i(ElectroBot.LOGTAG, "Red = " + R + ":: G = " + G + ":: Alpha = " + Alpha + " ColorSensor = " + colorsensor + "Total: " + (R + G + Alpha));
        //Log.i(ElectroBot.LOGTAG, "H: " + hsvValues[0] + " ,S: " + hsvValues[1] + " , V: " + hsvValues[2]);

        //return (R >= 10 && G >= 10 && Alpha >= 10);
        return (R + G + Alpha >= 30);
    }

    public double getColorSensorRatio() {

        double cs1;
        double cs2;
        int R1, R2;
        int G1, G2;
        int A1, A2;

        R1 = colorSensor1.red();
        G1 = colorSensor1.green();
        A1 = colorSensor1.alpha();

        R2 = colorSensor2.red();
        G2 = colorSensor2.green();
        A2 = colorSensor2.alpha();

        cs1 = R1 + G1 + A1;
        cs2 = R2 + G2 + A2;

        Log.i(ElectroBot.LOGTAG, "ColorValues: CS1: Total:: "+cs1+":: CS2:: "+cs2+":: Ratio:: "+(cs1/cs2));
        Log.i(ElectroBot.LOGTAG, "ColorValues: CS1:  Red :: " + R1 + ":: Green :: " + G1 + ":: Alpha :: " + A1);
        Log.i(ElectroBot.LOGTAG, "ColorValues: CS2:  Red :: " + R2 + ":: Green :: " + G2 + ":: Alpha :: " + A2);
        return cs1/cs2;

    }

    public boolean isGyroCalibrated() {
        return gyro.isGyroCalibrated();
    }

    private void resetHeading() {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        curHeading = 0.0;
    }

    public double[] currentdrivepower(){
        double[] dpow = new double[4];
        dpow[0] = fl.getPower();
        dpow[1] = fr.getPower();
        dpow[2] = bl.getPower();
        dpow[3] = br.getPower();

        return dpow;
    }

    public boolean isLiftMoving() {
        boolean retVal = false;
        if (llm.isBusy() && rlm.isBusy())
            retVal = true;

        return retVal;
    }

    public void turnCounterClockwise(double turnAngle, double power) {
        double error = 0.0;
        double steer = 0.0;
        double local_power = power;
        double t_coeff = 0.09;

        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetHeading();

        error = turnAngle - getHeading();
        while(error >= 0.5) {
            if (error/turnAngle > 0.3)
                local_power = power;
            else
            {
                steer = getSteer(error, t_coeff);
                local_power = power * steer;
            }

            //Log.i(ElectroBot.LOGTAG, "Counter clockwise power : " + local_power + "Error : " + error);
            this.setDriveMotorPower(-1 * local_power, local_power, -1 * local_power, local_power);
            error = turnAngle - getHeading();
        }
        this.setDriveMotorPower(0, 0, 0, 0);
        resetHeading();
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(10);
        } catch (Exception ex) {
            Log.e(LOGTAG, "Turn Counter Clockwise error: " + ex.toString());
        }
    }

    public void turnBiasCounterClockwise(int turnAngle, double lpower, double rpower) {
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetHeading();
        this.setDriveMotorPower(-1 * lpower, rpower, -1 * lpower, rpower);
        while (getHeading() < turnAngle) {

            //Log.i(LOGTAG, "Z: " + getHeading());
        }
        this.setDriveMotorPower(0, 0, 0, 0);
        resetHeading();
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(10);
        } catch (Exception ex) {
            Log.e(LOGTAG, "Turn Bias Counter Clockwise error: " + ex.toString());
        }
    }

    public void turnClockwise(double angle, double power) {
        double error = 0.0;
        double steer = 0.0;
        double local_power = power;
        double t_coeff = 0.09;

        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetHeading();

        error = angle + getHeading();
        while (error >= 0.5) {
            if (error / angle > 0.3)
                local_power = power;
            else {
                steer = getSteer(error, t_coeff);
                local_power = power * steer;
            }

            //Log.i(ElectroBot.LOGTAG, "Counter clockwise power : " + local_power + "Error : " + error);
            this.setDriveMotorPower(local_power, -1 * local_power, local_power, -1 * local_power);
            error = angle + getHeading();
        }

        /*this.setDriveMotorPower(power, -1 * power, power, -1 * power);

        while (getHeading() == 0.0) {
            //Log.i(LOGTAG, "Z: " + getHeading());
        }
        while (getHeading() > (-1 * angle)) {
            //Log.i(LOGTAG, "Z: " + getHeading());
        }*/
        this.setDriveMotorPower(0, 0, 0, 0);
        resetHeading();
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(10);
        } catch (Exception ex) {
            Log.e(LOGTAG, "Turn Clockwise error: " + ex.toString());
        }
    }

    public void turnBiasClockwise(int angle, double lpower, double rpower) {

        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetHeading();
        this.setDriveMotorPower(lpower, -1 * rpower, lpower, -1 * rpower);

        while (getHeading() == 0.0) {
            //Log.i(LOGTAG, "Z: " + getHeading());
        }
        while (getHeading() > (-1 * angle)) {
            //Log.i(LOGTAG, "Z: " + getHeading());
        }
        this.setDriveMotorPower(0, 0, 0, 0);
        resetHeading();
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(10);
        } catch (Exception ex) {
            Log.e(LOGTAG, "Turn Bias Clockwise error: " + ex.toString());
        }
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

        return curHeading;
    }

    public double getYaw() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
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

    public double getCountsPerInch() {
        return COUNTS_PER_INCH;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public enum DIRECTIONS {FORWARD, BACKWARD, LEFT, RIGHT}

}








