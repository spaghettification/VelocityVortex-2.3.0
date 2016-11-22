package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Please Work", group = "6994 Bot")
public class BlueButtonPusher extends HardwareMap {

    private final static int Encoder_CPR = 1120;
    private final static int WheelDiameter = 4;
    private final static double Circumference = Math.PI * WheelDiameter;
    private final static double Rotations = 1 / Circumference;
    private final static double CountsPerInch = Encoder_CPR * Rotations;
    private final BlueButtonPusherPathSeg[] mBeaconPath = {
            new BlueButtonPusherPathSeg(.5, 0, 24, DriveStyle.Linear),
            //new BlueButtonPusherPathSeg(.2, -45, 0, DriveStyle.CounterClockwise),
            /*new PathSeg(.2, 0, 6, DriveStyle.Linear),
            new PathSeg(.2, -90, 0, DriveStyle.Clockwise),
            new PathSeg(.2, 0, 6, DriveStyle.Linear),
            new PathSeg(.2, 0, 0, DriveStyle.CounterClockwise),
            new PathSeg(.2, 0, -6, DriveStyle.Linear),*/

    };
    private final ElapsedTime mRuntime = new ElapsedTime();
    private final ElapsedTime mStateTime = new ElapsedTime();
    public Color color;
    private int FrontLeftEncoderTarget;
    private int BackLeftEncoderTarget;
    private int FrontRightEncoderTarget;
    private int BackRightEncoderTarget;
    private int Heading;
    private State mCurrentState;
    private BlueButtonPusherPathSeg[] mCurrentPath;
    private int mCurrentSeg;
    private DriveStyle mDirection;

    public BlueButtonPusher() {
    }

    @Override
    public void init() {
        // Initialize class members.
        Gyro = hardwareMap.gyroSensor.get(gyroSensor);
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);

        Catapult = hardwareMap.dcMotor.get(catapult);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        setDrivePower(0, 0, 0, 0);

        initialize();
    }

    public void init_loop() {
        initialize();
    }

    @Override
    public void start() {
        setDriveSpeed(0, 0, 0, 0);
        mRuntime.reset();
        resetDriveEncoders();
        newState(State.STATE_INITIAL);


    }

    @Override
    public void loop() {
        telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());
        telemetry.addData("Integrated Z", getIntegratedZValue());
        telemetry.addData("Targetheading(?)", Heading);
        telemetry.addData("1", String.format("FL %5d - FR %5d - BL %5d - BR %5d", getFrontLeftPosition(),
                getFrontRightPosition(), getBackLeftPosition(), getBackRightPosition()));
        telemetry.addData("turnComplete", turnComplete());
        telemetry.addData("MoveComplete", MoveComplete());
        telemetry.addData("Direction", mDirection);

        switch (mCurrentState) {

            case STATE_INITIAL:{
                if (encodersAtZero() && !Gyro.isCalibrating()) {
                    startPath(mBeaconPath);
                    newState(State.STATE_DRIVE_TO_BEACON);
                }}
                break;
            case STATE_DRIVE_TO_BEACON:{
                if ((pathComplete())){newState(State.stop);}}


                break;
            case STATE_DRIVE_TO_MOUNTAIN:{

            }

                break;
            case stop: {
                Catapult.setPower(0);
                useConstantPower();
                setDrivePower(0, 0, 0, 0);
            }


        }
    }

    @Override
    public void stop() {
        useConstantPower();
        setDrivePower(0, 0, 0, 0);
    }

    private void newState(State newState) {
        mStateTime.reset();
        mCurrentState = newState;
    }

    private int getFrontLeftPosition() {
        return FrontLeft.getCurrentPosition();
    }

    private int getFrontRightPosition() {
        return FrontRight.getCurrentPosition();
    }

    private int getBackLeftPosition() {
        return BackLeft.getCurrentPosition();
    }

    private int getBackRightPosition() {
        return BackRight.getCurrentPosition();
    }

    public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    private void syncEncoders() {// initialize the publically accessable value "MotorEncoderTarget" for access through out the class.
        FrontLeftEncoderTarget = FrontLeft.getCurrentPosition();
        FrontRightEncoderTarget = FrontRight.getCurrentPosition();
        BackLeftEncoderTarget = BackLeft.getCurrentPosition();
        BackRightEncoderTarget = BackRight.getCurrentPosition();
    }

    private void setEncoderTarget(int FrontleftEncoder, int FrontrightEncoder, int BackleftEncoder, int BackrightEncoder) { // Changes the publically accessable "MoterEncoderTarget" to the target for access through out the class.
        FrontLeft.setTargetPosition(FrontLeftEncoderTarget = FrontleftEncoder);
        FrontRight.setTargetPosition(FrontRightEncoderTarget = FrontrightEncoder);
        BackLeft.setTargetPosition(BackLeftEncoderTarget = BackleftEncoder);
        BackRight.setTargetPosition(BackRightEncoderTarget = BackrightEncoder);
    }

    private void addEncoderTarget(int FrontleftEncoder, int FrontrightEncoder, int BackleftEncoder, int BackrightEncoder) { // Adds the new target to the old target for access through out the class./
        FrontLeft.setTargetPosition((FrontLeftEncoderTarget += FrontleftEncoder));
        FrontRight.setTargetPosition(FrontRightEncoderTarget += FrontrightEncoder);
        BackLeft.setTargetPosition(BackLeftEncoderTarget += BackleftEncoder);
        BackRight.setTargetPosition(BackRightEncoderTarget += BackrightEncoder);

    }

    private void syncHeading() {// initalize heading for access through out the class
        Heading = getIntegratedZValue();
    }

    private void setTargetHeading(int TargetHeading) { // change heading to target heading for access through out the class
        Heading = TargetHeading;
    }

    private void setDirection(DriveStyle direction) {// sets a new direction and makes said direction publically accessable. This is used for the TurnComplete and MoveComplete methods.
        mDirection = direction;
    }

    private void initializeDirection() {// Initializes the direction as linear to avoid any Null Pointer exceptions or out of bound array issues.
        mDirection = DriveStyle.Linear;
    }

    public void initialize() {
        initializeDirection();
        syncHeading();
        resetDriveEncoders();
    }

    public void calibrate() {
        resetDriveEncoders();
        Gyro.calibrate();

    }

    private void setDrivePower(double Frontleftpower, double Frontrightpower, double Backleftpower, double Backrightpower) { // Set power with clipped range to avoid any overloading.
        FrontLeft.setPower(Range.clip(Frontleftpower, -1, 1));
        FrontRight.setPower(Range.clip(Frontrightpower, -1, 1));
        BackLeft.setPower(Range.clip(Backleftpower, -1, 1));
        BackRight.setPower(Range.clip(Backrightpower, -1, 1));
    }

    private void setDriveSpeed(double Frontleft, double Frontright, double Backleft, double Backright) {
        setDrivePower(Frontleft, Frontright, Backleft, Backright);
    }

    private void PowerWithAngularAdjustment(double Power, double TargetAngle) {// Uses the gyro to drive straight
        double difference = (Math.abs(TargetAngle - Gyro.getHeading()));
        double Adjustment = 1.02 * difference;
        if (getIntegratedZValue() - TargetAngle < -1) {
            setDrivePower(Power / Adjustment, Power * Adjustment, Power / Adjustment, Power * Adjustment);
        } else if (getIntegratedZValue() - TargetAngle > 1) {
            setDrivePower(Power * Adjustment, Power / Adjustment, Power * Adjustment, Power / Adjustment);
        } else {
            setDrivePower(Power, Power, Power, Power);
        }
    }

    private void runToPosition() {
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void useConstantSpeed() {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void useConstantPower() {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetDriveEncoders() {
        setEncoderTarget(0, 0, 0, 0);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setDriveMode(DcMotor.RunMode mode) {// Ensure that motor drive mode is the correct moce and if not set the drive mode to the correct mode.
        if (FrontLeft.getMode() != mode)
            FrontLeft.setMode(mode);

        if (FrontRight.getMode() != mode)
            FrontRight.setMode(mode);
        if (BackLeft.getMode() != mode)
            BackLeft.setMode(mode);

        if (BackRight.getMode() != mode)
            BackRight.setMode(mode);
    }

    private boolean encodersAtZero() {// ensure that encoders have reset
        return ((Math.abs(getFrontLeftPosition()) < 5) && (Math.abs(getFrontRightPosition()) < 5) && (Math.abs(getBackLeftPosition()) < 5) && (Math.abs(getBackRightPosition()) < 5));
    }

    private void EncoderStop() {// Stop encoders when sensor has been activated and target position has been set to an unreachable number.
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition());
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition());
        BackLeft.setTargetPosition(BackLeft.getCurrentPosition());
        BackRight.setTargetPosition(BackRight.getCurrentPosition());
    }

    private void DriveWithEncoder(double Power, int TargetAngle, double Distance, DriveStyle Direction) {// use encoders and gyro drive and turn.
        double power = Range.clip(Power, 0, .2);
        setDirection(Direction);
        switch (mDirection) {
            case Linear: {
                int counts = (int) (Distance * CountsPerInch);
                setDrivePower(Power, Power, Power, Power);
                addEncoderTarget(counts, counts, counts, counts);
                setTargetHeading(TargetAngle);

            }
            break;
            case Clockwise: {
                setDrivePower(power, power, power, power);
                setTargetHeading(TargetAngle);

                if (Math.abs(getIntegratedZValue() - Heading) < 1.25) {
                    setEncoderTarget(getFrontLeftPosition(), getFrontRightPosition(), getBackLeftPosition(), getBackRightPosition());
                } else
                    addEncoderTarget(1000000, -1000000, 100000, -100000);// target position set to unreachable number to keep the drive mode as run to position constantly. once the target angle has been reached the encoder target will be set to the current position

            }

            break;
            case CounterClockwise: {
                setTargetHeading(TargetAngle);
                setDrivePower(power, power, power, power);
                if (Math.abs(getIntegratedZValue() - Heading) < 1.25) {
                    setEncoderTarget(getFrontLeftPosition(), getFrontRightPosition(), getBackLeftPosition(), getBackRightPosition());
                } else
                    addEncoderTarget(-1000000, 1000000, -100000, 100000);

            }
            break;

        }
    }

    private boolean turnComplete() {
        if (((Math.abs(getIntegratedZValue() - Heading) < 1.25) && mDirection != DriveStyle.Linear) && mDirection != DriveStyle.UltraSonic) {
            EncoderStop();
            syncEncoders();
        }
        return ((Math.abs(getIntegratedZValue() - Heading) < 1.25) && mDirection != DriveStyle.Linear && mDirection != DriveStyle.UltraSonic);
    }

    private boolean MoveComplete() {
        return ((Math.abs(FrontLeft.getCurrentPosition()-FrontLeft.getTargetPosition())<5)&&
                (Math.abs(FrontRight.getCurrentPosition()-FrontRight.getTargetPosition())<5)&&
                (Math.abs(BackLeft.getCurrentPosition()-BackLeft.getTargetPosition())<5)&&
                (Math.abs(BackRight.getCurrentPosition()-BackRight.getTargetPosition())<5)&&
                (mDirection ==DriveStyle.Linear));
        //return ((!FrontLeft.isBusy() && !FrontRight.isBusy() && !BackLeft.isBusy() && !BackRight.isBusy()) && (mDirection == DriveStyle.Linear));
    }

    private void startPath(BlueButtonPusherPathSeg[] path) {
        mCurrentPath = path;
        mCurrentSeg = 0;
        initialize();
        startSeg();
    }

    private void startSeg() {

        if (mCurrentPath != null) {
            DriveWithEncoder(mCurrentPath[mCurrentSeg].mpower, mCurrentPath[mCurrentSeg].mtargetangle, mCurrentPath[mCurrentSeg].mdistance, mCurrentPath[mCurrentSeg].mdirection);
            runToPosition();
            mCurrentSeg++;
        }
    }

    private boolean pathComplete() {

        if (MoveComplete() || turnComplete()) {


            if (mCurrentSeg < mCurrentPath.length) {
                startSeg();
                runToPosition();
            } else {
                mCurrentPath = null;
                mCurrentSeg = 0;
                initialize();
                setDriveSpeed(0, 0, 0, 0);
                useConstantSpeed();
                return true;
            }


        }
        return false;
    }


    private enum State {
        STATE_INITIAL,
        STATE_DRIVE_TO_BEACON,
        STATE_FOLLOW_LINE,
        STATE_SQUARE_TO_WALL,
        STATE_DEPLOY_CLIMBERS,
        STATE_DRIVE_TO_MOUNTAIN,
        STATE_CLIMB_MOUNTAIN,
        stop,
    }

    enum DriveStyle {Linear, Clockwise, CounterClockwise, UltraSonic}

    public enum Color {Red, Blue, Green}
}

class BlueButtonPusherPathSeg {
    double mpower;
    int mtargetangle;
    double mdistance;
    BlueButtonPusher.DriveStyle mdirection;

    public BlueButtonPusherPathSeg(double Power, int TargetAngle, double Distance, BlueButtonPusher.DriveStyle direction) {
        mpower = Power;
        mtargetangle = TargetAngle;
        mdistance = Distance;
        mdirection = direction;
    }
}
