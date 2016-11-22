package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Trevor on 11/5/2016.
 */
public abstract class LinearHardwareMap extends LinearOpMode {
   public DcMotor                                         FrontLeft;
   public DcMotor                                         FrontRight;
   public DcMotor                                         BackLeft;
   public DcMotor                                         BackRight;
   public DcMotor                                         Catapult;
   public DcMotor                                         BallCollection;
   public DcMotor                                         CapBallLiftLeft;
   public DcMotor                                         CapBallLiftRight;

   public GyroSensor                                      Gyro;

   public ColorSensor                                     BeaconColorSensor;
   public ColorSensor                                     WhiteLineFinder;

   public ModernRoboticsI2cRangeSensor                    SideRangeSensor;
   public ModernRoboticsI2cRangeSensor                    FrontRangeSensor;
   public ModernRoboticsI2cRangeSensor                    BackRangeSensor;

   public TouchSensor CatapultStop;

   public Servo                                           ButtonPusherLeft;
   public Servo                                           ButtonPusherRight;
   public Servo                                           CapBallFork;

   public Servo                                           servo6;
   public Servo                                           servo7;
   public Servo                                           servo8;
   public Servo                                           servo9;
   public Servo                                           servo10;
   public Servo                                           servo11;
   public Servo                                           servo12;

   public CRServo                                         CapBallarm1;
   public CRServo                                         CapBallarm2;
   public Servo                                           BallControl;


   public DeviceInterfaceModule                           DIM;


    public String frontLeftMotor                    = "fl";
    public String frontRightMotor                   = "fr";
    public String backLeftMotor                     = "bl";
    public String backRightMotor                    = "br";
    public String catapult                          = "c";
    public String ballCollection                    = "bc";
    public String capBallLiftLeft                   = "cbll";
    public String capBallLiftRight                  = "cblr";
    public String gyroSensor                        = "gyro";
    public String beaconColorSensor                 = "bcs";
    public String whiteLineFinder                   = "wlf";
    public String sideRangeSensor                   = "brsrs";
    public String frontRangeSensor                  = "brrs";
    public String backRangeSensor                   = "blrs";
    public String catapultStop                      = "cs";
    public String capBallFork                       = "cbf";
    public String capBallarm1                       = "cba1";
    public String capBallarm2                       = "cba2";
    public String ballControll                      = "ballco";
    public String buttonPusherLeft                  = "bpl";
    public String buttonPusherRight                 = "bpr";



    public double buttonPusherLeftStartPositoin     = 0;
    public double buttonPusherRightStartPositoin    = 1;
    public double buttonPusherLeftEngagedPositoin   = 0;
    public double buttonPusherRightEngagedPositoin  = 1;
    public double ballControlStartPosition          = 1;
    public double ballControlEngagedPosition        = 0;

    public float LinearproportionalConstant         = 0;
    public float LinearintegralConstant             = 0;
    public float LinearderivitiveConstant           = 0;
    public float AngularproportionalConstant        = 0;
    public float AngularintegralConstant            = 0;
    public float AngularderivitiveConstant          = 0;

    public double AndyMarkMotor_TicksPerRevolution  = 1120;
    public double CountsPerInch=(AndyMarkMotor_TicksPerRevolution/4*Math.PI);

    String VuforiaLicenseKey = "AbkJpf//////AAAAGfwmmKkkGUDwrRcXe4puyLQhZ3m1wmsmuJUw2GVDtb7tWinUTnSd+UmyGz5aylC8ShWX8ayvA9h2mDtWnM1s3yni7S/WtH8buZO7gUBz9FotxNPJGL8Di9VJSmOhzEoyHLivQpx/vPwoH0Aejcvr1lBt8b5yMEgegLQ+WbmwNmj25ciaaMFDhryp7CTOzZFswvIUdhZ84PBJJew94ewMFjrsGNqra+0beno8wvEH9XmHp2kj9lVT+u8EjZdSQuEowkS5Lw2bnmOCMfPk9/00KZ+xBfaa2LDB3IXuYR2FVdd6qORTWXA8N120mYbCx8x8U7R4JdZs/eAH279CtHqFyFPdQtj3qn3Of7Z3urbcezNu";


    public float AngularMaxCorrection               = 100;
    public float AngularMinCorrection               = 15;
    public float LinearMaxCorrection                = 100;
    public float LinearMinCorrection                = 15;/*

    public String Dim                               = "DeviceInterfaceModule1";


    public VuforiaLocalizer vuforiaLocalizer;
    public VuforiaLocalizer.Parameters parameters;
    public VuforiaTrackables visionTargets;

    public VuforiaTrackableDefaultListener Wheelslistener;
    public VuforiaTrackableDefaultListener Gearslistener;
    public VuforiaTrackableDefaultListener Toolslistener;
    public VuforiaTrackableDefaultListener Legoslistener;

    private VuforiaTrackable Wheels;
    private VuforiaTrackable Gears;
    private VuforiaTrackable Tools;
    private VuforiaTrackable Legos;

    public OpenGLMatrix lastKnownLocation;
    public OpenGLMatrix phoneLocation;

    float mmPerInch        = 25.4f;
*/
    public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    public void StopAllMotors(){
        if (FrontLeft != null){
        FrontLeft.setPower(0);
        }
        if (FrontRight != null){
        FrontRight.setPower(0);
        }
        if (BackRight != null){
        BackLeft.setPower(0);
        }
        if (BackRight != null){
        BackRight.setPower(0);
        }
        if (Catapult != null){
            Catapult.setPower(0);
        }
        if (BallCollection != null){
            BallCollection.setPower(0);
        }
        if(CapBallLiftRight != null){
            CapBallLiftRight.setPower(0);
        }
        if(CapBallLiftLeft != null){
            CapBallLiftLeft.setPower(0);
        }
    }

    public void Calibrate(){
        if(Gyro != null){
            Gyro.calibrate();
        }
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void AutonomousHardwareMap(){
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);

        Catapult = hardwareMap.dcMotor.get(catapult);
        BallCollection = hardwareMap.dcMotor.get(ballCollection);
        CapBallLiftLeft = hardwareMap.dcMotor.get(capBallLiftLeft);
        CapBallLiftRight = hardwareMap.dcMotor.get(capBallLiftRight);

        CatapultStop = hardwareMap.touchSensor.get(catapultStop);

        BallControl = hardwareMap.servo.get(ballControll);

        SideRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, sideRangeSensor);
        FrontRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, frontRangeSensor);
        BackRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, backRangeSensor);

        Gyro = hardwareMap.gyroSensor.get(gyroSensor);

        BeaconColorSensor = hardwareMap.colorSensor.get(beaconColorSensor);
        WhiteLineFinder = hardwareMap.colorSensor.get(whiteLineFinder);}

    public void InitializeServoPositions(){
        if(BallControl != null){
            BallControl.setPosition(ballControlStartPosition);
        }
        if (ButtonPusherLeft != null){
            ButtonPusherLeft.setPosition(buttonPusherLeftStartPositoin);
        }
        if (ButtonPusherRight != null){
            ButtonPusherRight.setPosition(buttonPusherRightStartPositoin);
        }

    }



    public void setPower(double FL, double FR, double BL, double BR) {
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR);
        BackLeft.setPower(BL);
        BackRight.setPower(BR);
    }

    public void SetMode(DcMotor.RunMode mode) {
        FrontLeft.setMode(mode);
        FrontRight.setMode(mode);
        BackLeft.setMode(mode);
        BackRight.setMode(mode);
    }

    public void setMaxSpeed(int TicksPerSecond) {
        FrontLeft.setMaxSpeed(TicksPerSecond);
        FrontRight.setMaxSpeed(TicksPerSecond);
        BackLeft.setMaxSpeed(TicksPerSecond);
        BackRight.setMaxSpeed(TicksPerSecond);
    }

    public void Drive(double minPower, int Distance, int TargetAngle, double TimeOut, boolean PIDdesired) {
        double FrontLeftDynamicPower;
        double FrontRightDynamicPower;
        double BackLeftDynamicPower;
        double BackRightDynamicPower;
        int AngleToMaintain = getIntegratedZValue();

        sleep(200);
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(50);
        double EncoderTicks = 89.17*Distance;
        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition() + EncoderTicks));
        FrontRight.setTargetPosition((int) (FrontRight.getTargetPosition() + EncoderTicks));
        BackLeft.setTargetPosition((int) (BackLeft.getCurrentPosition() + EncoderTicks));
        BackRight.setTargetPosition((int) (BackRight.getTargetPosition() + EncoderTicks));
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setPower(minPower);
        FrontRight.setPower(minPower);
        BackLeft.setPower(minPower);
        BackRight.setPower(minPower);

        while ((opModeIsActive() &&
                FrontLeft.isBusy() &&
                FrontRight.isBusy())
                ) {
            idle();
        }
        setPower(0, 0, 0, 0);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(300);
    }

    public void Turn(double Power, int TargetAngle, boolean FrontWheelPivot) {
        double FrontLeftTurnPower = 0;
        double FrontRightTurnPower = 0;
        double BackLeftTurnPower = 0;
        double BackRightTurnPower = 0;
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(300);
        if (FrontWheelPivot) {
            if (TargetAngle > getIntegratedZValue()) {
                FrontLeftTurnPower = Power;
                FrontRightTurnPower = -Power;
                BackLeftTurnPower = Power;
                BackRightTurnPower = -Power;

            } else if (TargetAngle < getIntegratedZValue()) {
                FrontLeftTurnPower = -Power;
                FrontRightTurnPower = Power;
                BackLeftTurnPower = -Power;
                BackRightTurnPower = Power;
            } else {
                setPower(0, 0, 0, 0);
            }
            do {
                setPower(FrontLeftTurnPower, FrontRightTurnPower, 0,0);
                idle();
                telemetry.addData(">", "Turning!");
                sleep(50);

            }
            while (Math.abs(getIntegratedZValue() - TargetAngle) > 1.25);
            {
                setPower(FrontLeftTurnPower, FrontRightTurnPower, 0,0);
                idle();
                telemetry.addData(">", "Turning!");
                sleep(50);

            }
            setPower(0, 0, 0, 0);

        }
        else{
        if (TargetAngle > getIntegratedZValue()) {
            FrontLeftTurnPower = Power;
            FrontRightTurnPower = -Power;
            BackLeftTurnPower = Power;
            BackRightTurnPower = -Power;

        } else if (TargetAngle < getIntegratedZValue()) {
            FrontLeftTurnPower = -Power;
            FrontRightTurnPower = Power;
            BackLeftTurnPower = -Power;
            BackRightTurnPower = Power;
        } else {
            setPower(0, 0, 0, 0);
        }
        do {
            setPower(FrontLeftTurnPower, FrontRightTurnPower, BackLeftTurnPower, BackRightTurnPower);
            idle();
            telemetry.addData(">", "Turning!");
            sleep(50);

        }
        while (Math.abs(getIntegratedZValue() - TargetAngle) > 1.25);
        {
            setPower(FrontLeftTurnPower, FrontRightTurnPower, BackLeftTurnPower, BackRightTurnPower);
            idle();
            telemetry.addData(">", "Turning!");
            sleep(50);

        }
        setPower(0, 0, 0, 0);
    }}
    public void DriveToWall(){
        while (FrontRangeSensor.getDistance(DistanceUnit.INCH) > 4) {
            setPower(.125, .125, .125, .125);
        }setPower(0,0,0,0);
    }

    public void ButtonPush(String Color) {
        if (Color.toLowerCase() == "blue") {
            while (WhiteLineFinder.red() > 180 && WhiteLineFinder.blue() > 180 && WhiteLineFinder.green() > 180) {
                setPower(.125, .125, .125, .125);
            }
            Turn(.125, 90, true);
            sleep(300);
            DriveToWall();
            if (BeaconColorSensor.blue() > BeaconColorSensor.red() && BeaconColorSensor.blue() > BeaconColorSensor.green()) {
                ButtonPusherLeft.setPosition(buttonPusherLeftEngagedPositoin);
                sleep(500);
                ButtonPusherLeft.setPosition(buttonPusherLeftStartPositoin);
            } else if (BeaconColorSensor.red() > BeaconColorSensor.blue() && BeaconColorSensor.red() > BeaconColorSensor.green()) {
                ButtonPusherRight.setPosition(buttonPusherRightEngagedPositoin);
                sleep(500);
                ButtonPusherRight.setPosition(buttonPusherRightStartPositoin);
            } else {
                telemetry.addData(">", "Color Sensor Did not Find The Beacon");
                requestOpModeStop();
            }
        } else {
            while (WhiteLineFinder.red() > 180 && WhiteLineFinder.blue() > 180 && WhiteLineFinder.green() > 180) {
                setPower(.125, .125, .125, .125);
            }
            Turn(.125, -90, true);
            sleep(300);
            while (FrontRangeSensor.getDistance(DistanceUnit.INCH) > 4) {
                setPower(.125, .125, .125, .125);
            }
            setPower(0, 0, 0, 0);
            if (BeaconColorSensor.red() > BeaconColorSensor.blue() && BeaconColorSensor.red() > BeaconColorSensor.green()) {
                ButtonPusherLeft.setPosition(buttonPusherLeftEngagedPositoin);
                sleep(500);
                ButtonPusherLeft.setPosition(buttonPusherLeftStartPositoin);
            } else if (BeaconColorSensor.blue() > BeaconColorSensor.red() && BeaconColorSensor.blue() > BeaconColorSensor.green()) {
                ButtonPusherRight.setPosition(buttonPusherRightEngagedPositoin);
                sleep(500);
                ButtonPusherRight.setPosition(buttonPusherRightStartPositoin);
            } else {
                telemetry.addData(">", "Color Sensor Did not Find The Beacon");
                requestOpModeStop();
            }
        }

    }
    /*

    public void InitializeVuforia(){
        parameters=new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey=VuforiaLicenseKey;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer= ClassFactory.createVuforiaLocalizer(parameters);
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016_17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        Wheels = visionTargets.get(0);
        Wheels.setName("Wheels Target");
        Wheels.setLocation(createMatrix(144*mmPerInch,60*mmPerInch,6*mmPerInch,0,0,0));

        Gears = visionTargets.get(1);
        Gears.setName("Gears Target");
        Gears.setLocation(createMatrix(60*mmPerInch,144*mmPerInch,6*mmPerInch,0,0,0));

        Tools = visionTargets.get(2);
        Tools.setName("Tools Target");
        Tools.setLocation(createMatrix(60+48*mmPerInch,144*mmPerInch,6*mmPerInch,0,0,0));

        Legos = visionTargets.get(3);
        Legos.setName("Legos Target");
        Legos.setLocation(createMatrix(144*mmPerInch,60+48*mmPerInch,0*mmPerInch,0,0,0));

        phoneLocation = createMatrix(9*mmPerInch,9*mmPerInch,14*mmPerInch,0,0,0);

        Wheelslistener = (VuforiaTrackableDefaultListener) Wheels.getListener();
        Wheelslistener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

        Gearslistener = (VuforiaTrackableDefaultListener) Gears.getListener();
        Gearslistener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

        Toolslistener = (VuforiaTrackableDefaultListener) Tools.getListener();
        Toolslistener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

        Legoslistener = (VuforiaTrackableDefaultListener) Legos.getListener();
        Legoslistener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

        lastKnownLocation = createMatrix(0,0,0,0,0,0);
    }
    OpenGLMatrix LatestLocation =null;
    double RobotX=0;
    double RobotY=0;
    double RobotTheta=0;
    float Output = Float.parseFloat(null);

    public double RobotLocationOnField(String LookingFor, String Tracking){

        switch(Tracking.toLowerCase()){
            case "wheels":{
                LatestLocation=Wheelslistener.getUpdatedRobotLocation();
            }break;
            case "gears":{
                LatestLocation=Gearslistener.getUpdatedRobotLocation();
            }break;
            case "tools":{
                LatestLocation=Toolslistener.getUpdatedRobotLocation();
            }break;
            case "legos":{
                LatestLocation=Legoslistener.getUpdatedRobotLocation();
            }break;
        }
        if (LatestLocation !=null){
            LatestLocation = lastKnownLocation;}
        float[] Coordinates = lastKnownLocation.getTranslation().getData();

        RobotX = Coordinates[0];
        RobotY = Coordinates[1];
        RobotTheta = Orientation.getOrientation(lastKnownLocation,AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;


                switch (LookingFor.toLowerCase()){
                    case"x":{
                        Output= (float) RobotX;
                    }break;
                    case"y":{
                        Output = (float) RobotY;
                    }break;
                    case"theta":{
                        Output = (float) RobotTheta;
                    }break;
                }
        return Output;
        }

    public void VuforiaTelemetry(){
        telemetry.addData("Tracking"+Wheels.getName(),Wheelslistener.isVisible());
        telemetry.addData("Last Known Loacation", formatMatrix(lastKnownLocation));
        telemetry.update();
    }

    public OpenGLMatrix createMatrix(float x,float y,float z,float u,float v,float w){
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC,
                AxesOrder.XYZ, AngleUnit.DEGREES,u,v,w));
    }
    public String formatMatrix(OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }*/



}
