package org.firstinspires.ftc.teamcode;

import android.provider.CalendarContract;
import android.widget.SeekBar;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Admin on 8/14/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Template", group = "6994 Bot")@Disabled
public class FTC_6994_Template extends HardwareMap{

    public final static int Encoder_CPR = 1440;
    public final static int WheelDiameter = 4;
    public final static double Circumference = Math.PI * WheelDiameter;
    public final static double Rotations = 1 / Circumference;
    public final static double CountsPerInch = Encoder_CPR * Rotations;
    public final ElapsedTime mRuntime = new ElapsedTime();
    public final ElapsedTime mStateTime = new ElapsedTime();
    public ModernRoboticsI2cRangeSensor Rangesensor;
    public ModernRoboticsAnalogOpticalDistanceSensor BackLeftODS;
    public ModernRoboticsAnalogOpticalDistanceSensor BackRightODS;
    public ModernRoboticsAnalogOpticalDistanceSensor RightFrontODS;
    public ModernRoboticsAnalogOpticalDistanceSensor RightBackODS;
    public Servo ButtonPusherLeft;
    public Servo ButtonPusherRight;
    public ColorSensor BeaconCS;
    public ColorSensor TapeCS;
    public TouchSensor CatapultStop;
    public GyroSensor Gyro;
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public int FrontLeftEncoderTarget;
    public int BackLeftEncoderTarget;
    public int FrontRightEncoderTarget;
    public int BackRightEncoderTarget;
    public double Heading;
    public PathSeg[] mCurrentPath;
    public int mCurrentSeg;
    public DriveStyle mdirection;
    public double UltrasonicTarget;
    public Color color;
    int TapeDistance;
    int XTolerance;
    int HTolerance;
    // experimental values for angular adjustment
    double ThetaPowerConstant = 1.5;
    double A = .8;
    int Reverse=-1;
    int Forward=1;
    int LeftButton = 1;
    int RightButton = 0;
    double ReflectancetoCM = 1;
    double ServoPositionBallRelease=1;
    double ServoPositionBallRestrict=.5;
    private DriveStyle mDirection;
    public enum Color {Red, Blue, Green}
    public enum CatapultStatus {Prime, Launch}

    public final PathSeg[] GyroTest = {new PathSeg(.25, 0, 24, DriveStyle.Linear)};
    @Override
    public void init() {
        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        Gyro = hardwareMap.gyroSensor.get("G");
        resetDriveEncoders();
        useConstantSpeed();
        Gyro.calibrate();
    }

    public void init_loop() {
        useConstantSpeed();


    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (!Gyro.isCalibrating()){
        startPath(GyroTest);}
    }

    @Override
    public void stop() {
        useConstantPower();
        setDrivePower(0, 0, 0, 0);
    }

    public boolean FrontRed() {
        return ((BeaconCS.red() > BeaconCS.blue()) && (BeaconCS.red() > BeaconCS.green()));
    }

    public boolean FrontBlue() {
        return ((BeaconCS.blue() > BeaconCS.red()) && (BeaconCS.blue() > BeaconCS.green()));
    }

    public boolean FrontGreen() {
        return ((BeaconCS.green() > BeaconCS.blue()) && (BeaconCS.green() > BeaconCS.red()));
    }

    public void DriveUntil(Color mcolor, double Power) {
        switch (mcolor) {
            case Red: {
                setDrivePower(Power, Power, Power, Power);
                if (FrontRed()) {
                    setDrivePower(0, 0, 0, 0);
                }
            }
            case Blue: {
                setDrivePower(Power, Power, Power, Power);
                if (FrontBlue()) {
                    setDrivePower(0, 0, 0, 0);
                }
            }
            case Green: {
                setDrivePower(Power, Power, Power, Power);
                if (FrontGreen()) {
                    setDrivePower(0, 0, 0, 0);
                }
            }


        }

    }

    public void setupMotor(DcMotor motor, float Power, DcMotorSimple.Direction direction){
        motor.setPower(scaleInput(Power));
        motor.setDirection(direction);
    }

    public void PushButton(TeamColor color, Servo servo) {
        if ((FrontBlue() && color == TeamColor.Blue) || (FrontRed() && color == TeamColor.Red)) {
            servo.setPosition(LeftButton);
        } else if ((!FrontBlue() && color != TeamColor.Blue) || (!FrontRed() && color != TeamColor.Red)) {
            servo.setPosition(RightButton);
        } else {
            servo.setPosition(.5);
        }

    }

    public int getFrontLeftPosition() {
        return FrontLeft.getCurrentPosition();
    }

    public int getFrontRightPosition() {
        return FrontRight.getCurrentPosition();
    }

    public int getBackLeftPosition() {
        return BackLeft.getCurrentPosition();
    }

    public int getBackRightPosition() {
        return BackRight.getCurrentPosition();
    }

    public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    public void syncEncoders() {// initialize the publically accessable value "MotorEncoderTarget" for access through out the class.
        FrontLeftEncoderTarget = FrontLeft.getCurrentPosition();
        FrontRightEncoderTarget = FrontRight.getCurrentPosition();
        BackLeftEncoderTarget = BackLeft.getCurrentPosition();
        BackRightEncoderTarget = BackRight.getCurrentPosition();
    }

    public double CalculateHypotenuse(double X) {
        return Math.sqrt(((X - XTolerance) * (X - XTolerance)) + ((TapeDistance - 2 * HTolerance) + (TapeDistance - 2 * HTolerance)));
    }

    public double CalculateTheta(double X) {
        return (90 - (1 / Math.tan((X - XTolerance) / TapeDistance - 2 * HTolerance)));
    }

    public void setEncoderTarget(int FrontleftEncoder, int FrontrightEncoder, int BackleftEncoder, int BackrightEncoder) { // Changes the publically accessable "MoterEncoderTarget" to the target for access through out the class.
        FrontLeft.setTargetPosition(FrontLeftEncoderTarget = FrontleftEncoder);
        FrontRight.setTargetPosition(FrontRightEncoderTarget = FrontrightEncoder);
        BackLeft.setTargetPosition(BackLeftEncoderTarget = BackleftEncoder);
        BackRight.setTargetPosition(BackRightEncoderTarget = BackrightEncoder);
    }

    public void addEncoderTarget(int FrontleftEncoder, int FrontrightEncoder, int BackleftEncoder, int BackrightEncoder) { // Adds the new target to the old target for access through out the class./
        FrontLeft.setTargetPosition((FrontLeftEncoderTarget += FrontleftEncoder));
        FrontRight.setTargetPosition(FrontRightEncoderTarget += FrontrightEncoder);
        BackLeft.setTargetPosition(BackLeftEncoderTarget += BackleftEncoder);
        BackRight.setTargetPosition(BackRightEncoderTarget += BackrightEncoder);

    }

    public void SetUltrasonicTarget(double Distance) {
        UltrasonicTarget = Distance;
    }

    public boolean UltraSonicTargetHit(int Target) {
        return (UltrasonicTarget < Target);
    }

    public void syncHeading() {// initalize heading for access through out the class
        Heading = getIntegratedZValue();
    }

    public void setTargetHeading(double TargetHeading) { // change heading to target heading for access through out the class
        Heading = TargetHeading;
    }

    public void setDirection(DriveStyle direction) {// sets a new direction and makes said direction publically accessable. This is used for the TurnComplete and MoveComplete methods.
        mDirection = direction;
    }

    public void initializeDirection() {// Initializes the direction as linear to avoid any Null Pointer exceptions or out of bound array issues.
        mDirection = DriveStyle.Linear;
    }

    public void initialize() {
        initializeDirection();
        //InitializeUltrasonicLevel();
        syncEncoders();
        syncHeading();
        runToPosition();
    }

    public void calibrate() {
        resetDriveEncoders();
        Gyro.calibrate();

    }

    public void setDrivePower(double Frontleftpower, double Frontrightpower, double Backleftpower, double Backrightpower) { // Set power with clipped range to avoid any overloading.
        FrontLeft.setPower(Range.clip(Frontleftpower, -1, 1));
        FrontRight.setPower(Range.clip(Frontrightpower, -1, 1));
        BackLeft.setPower(Range.clip(Backleftpower, -1, 1));
        BackRight.setPower(Range.clip(Backrightpower, -1, 1));
    }

    public void setDriveSpeed(double Frontleft, double Frontright, double Backleft, double Backright) {
        setDrivePower(Frontleft, Frontright, Backleft, Backright);
    }

        public void PowerWithAngularAdjustment(double Power, double TargetAngle) {// Uses the gyro to drive straight
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

    public void runToPosition() {
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void useConstantSpeed() {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void useConstantPower() {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetDriveEncoders() {
        setEncoderTarget(0, 0, 0, 0);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDriveMode(DcMotor.RunMode mode) {// Ensure that motor drive mode is the correct moce and if not set the drive mode to the correct mode.
        if (FrontLeft.getMode() != mode)
            FrontLeft.setMode(mode);

        if (FrontRight.getMode() != mode)
            FrontRight.setMode(mode);
        if (BackLeft.getMode() != mode)
            BackLeft.setMode(mode);

        if (BackRight.getMode() != mode)
            BackRight.setMode(mode);
    }

    public boolean encodersAtZero() {// ensure that encoders have reset
        return ((Math.abs(getFrontLeftPosition()) < 5) && (Math.abs(getFrontRightPosition()) < 5) && (Math.abs(getBackLeftPosition()) < 5) && (Math.abs(getBackRightPosition()) < 5));
    }

    public void EncoderStop() {// Stop encoders wehn sensor has been activated and target position has been set to an unreachable number.
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition());
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition());
        BackLeft.setTargetPosition(BackLeft.getCurrentPosition());
        BackRight.setTargetPosition(BackRight.getCurrentPosition());
    }

    public void DriveWithEncoder(double Power, double TargetAngle, double Distance, DriveStyle Direction) {// use encoders and gyro drive and turn.
        double TargetRealityDifference = (TargetAngle-getIntegratedZValue());
        double TargetRealityDisplacement = Math.abs(TargetAngle-getIntegratedZValue());
        double power = Range.clip(Power, 0, .2);
        int FrontLeftCP= FrontLeft.getCurrentPosition();
        int FrontRighttCP= FrontRight.getCurrentPosition();
        int BackLeftCP= BackLeft.getCurrentPosition();
        int BackRightCP= BackRight.getCurrentPosition();
        setDirection(Direction);
        switch (mDirection) {
            case Linear: {
                int Counts = (int) (Distance * CountsPerInch);
                while(FrontLeft.getCurrentPosition()<Counts+FrontLeftCP&&FrontRight.getCurrentPosition()<Counts+FrontRighttCP&&BackLeft.getCurrentPosition()<Counts+BackLeftCP&&BackRight.getCurrentPosition()<Counts+BackRightCP)
                {setDrivePower(Power, Power, Power, Power);
                    setTargetHeading(TargetAngle);}
                setDrivePower(0,0,0,0);



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
            case FindWhiteLine: {
                setTargetHeading(TargetAngle);
                setDrivePower(power, power, power, power);
                while (!WhiteLineFound()) {
                    if (TargetRealityDifference > 0) {

                        setDrivePower(Power + (TargetRealityDisplacement * ThetaPowerConstant), Power - (TargetRealityDisplacement * ThetaPowerConstant), Power + (TargetRealityDisplacement * ThetaPowerConstant), Power - (TargetRealityDisplacement * ThetaPowerConstant));
                        addEncoderTarget(1000000, 1000000, 1000000, 1000000);
                        setTargetHeading(TargetAngle);

                    } else if (TargetRealityDifference < 0) {

                        setDrivePower(Power - (TargetRealityDisplacement * ThetaPowerConstant), Power + (TargetRealityDisplacement * ThetaPowerConstant), Power - (TargetRealityDisplacement * ThetaPowerConstant), Power + (TargetRealityDisplacement * ThetaPowerConstant));
                        addEncoderTarget(1000000, 1000000, 1000000, 1000000);
                        setTargetHeading(TargetAngle);

                    } else {

                        setDrivePower(Power, Power, Power, Power);
                        addEncoderTarget(1000000, 1000000, 1000000, 1000000);
                        setTargetHeading(TargetAngle);
                    }
                    if (WhiteLineFound()) {
                        setEncoderTarget(getFrontLeftPosition(), getFrontRightPosition(), getBackLeftPosition(), getBackRightPosition());
                        ;
                    }
                }}
            break;
            case SquareWithBack: {
                setTargetHeading(TargetAngle);
                setDrivePower(power, power, power, power);
                SetUltrasonicTarget(Distance);
                if (ODStoInch(RightFrontODS) == ODStoInch(RightBackODS)) {
                    EncoderStop();
                    syncEncoders();
                } else {
                    if (ODStoInch(RightFrontODS) > ODStoInch(RightBackODS)) {
                        addEncoderTarget(0, -100000, 0, 0);
                    } else {
                        addEncoderTarget(0, 0, 0, -100000);
                    }
                }
            }
            break;
            case SquareWithSide: {
                setTargetHeading(TargetAngle);
                setDrivePower(power, power, power, power);
                SetUltrasonicTarget(Distance);
                if (ODStoInch(BackLeftODS) == ODStoInch(BackRightODS)) {
                    EncoderStop();
                    syncEncoders();
                } else {
                    if (ODStoInch(BackLeftODS) > ODStoInch(BackRightODS)) {
                        addEncoderTarget(-100000, 0, 0, 0);
                    } else {
                        addEncoderTarget(0, 0, -100000, 0);
                    }
                }
            }
            break;
        }
    }

    public double ODStoInch(ModernRoboticsAnalogOpticalDistanceSensor Ods) {
        return (2.54 * Ods.getLightDetected() * ReflectancetoCM);
    }

    public boolean WhiteLineFound() {
        if (TapeCS.equals(0x08)){return true;}
        else return false;
    }

    public boolean turnComplete() {
        if (((Math.abs(getIntegratedZValue() - Heading) < 1.25) && mDirection != DriveStyle.Linear) && mDirection != DriveStyle.FindWhiteLine) {
            EncoderStop();
            syncEncoders();
        }
        return ((Math.abs(getIntegratedZValue() - Heading) < 1.25) && mDirection != DriveStyle.Linear && mDirection != DriveStyle.FindWhiteLine);
    }

    public boolean MoveComplete() {
        return ((
                Math.abs(FrontLeft.getCurrentPosition()-FrontLeft.getTargetPosition())<5)&&
                (Math.abs(FrontRight.getCurrentPosition()-FrontRight.getTargetPosition())<5)&&
        (Math.abs(BackLeft.getCurrentPosition()-BackLeft.getTargetPosition())<5)&&
        (Math.abs(BackRight.getCurrentPosition()-BackRight.getTargetPosition())<5)
                ) && (mDirection == DriveStyle.Linear);
    }

    public void startPath(PathSeg[] path) {
        mCurrentPath = path;
        mCurrentSeg = 0;
        initialize();
        startSeg();
    }

    public void startSeg() {

        if (mCurrentPath != null) {
            DriveWithEncoder(mCurrentPath[mCurrentSeg].mpower, mCurrentPath[mCurrentSeg].mtargetangle, mCurrentPath[mCurrentSeg].mdistance, mCurrentPath[mCurrentSeg].mdirection);
            runToPosition();
            mCurrentSeg++;
        }
    }

    public boolean pathComplete() {

        if (MoveComplete() || turnComplete() /*|| UltrasonicTargetHit()*/) {
            useConstantSpeed();


            if (mCurrentSeg < mCurrentPath.length) {
                startSeg();
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

   public double scaleInput(double dVal) {
        //first few numbers are zero to account for offest of joystick keeping motors at zero power until acted upon by driver
        double[] scaleArray = {0.0, 0.0, 0.2, 0.22, 0.28, 0.35, 0.40,
                0.45, 0.50, 0.55, 0.60, 0.65, 0.72, 0.85, .90, 1.0, 1.0
        };
        int index = (int) (dVal * 15.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 15) {
            index = 15;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        if (dVal < -1) {
            dVal = -1;
        }

        if (dVal > 1) {
            dVal = 1;
        }


        return dScale;
    }

    public enum TeamColor {Red, Blue}

    enum DriveStyle {Linear, Clockwise, CounterClockwise, FindWhiteLine, SquareWithBack, SquareWithSide}


}

class PathSeg {
    double mpower;
    double mtargetangle;
    double mdistance;
    FTC_6994_Template.DriveStyle mdirection;

    public PathSeg(double Power, double TargetAngle, double Distance, FTC_6994_Template.DriveStyle direction) {
        mpower = Power;
        mtargetangle = TargetAngle;
        mdistance = Distance;
        mdirection = direction;
    }
}

