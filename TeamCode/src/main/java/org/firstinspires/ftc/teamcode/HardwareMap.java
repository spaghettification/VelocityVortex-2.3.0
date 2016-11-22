package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Trevor on 11/5/2016.
 */
public abstract class HardwareMap extends OpMode {
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

    public float LinearproportionalConstant         =0;
    public float LinearintegralConstant             =0;
    public float LinearderivitiveConstant           =0;
    public float AngularproportionalConstant        =0;
    public float AngularintegralConstant            =0;
    public float AngularderivitiveConstant          =0;

    public double AndyMarkMotor_TicksPerRevolution  =1120;
    public double CountsPerInch=(AndyMarkMotor_TicksPerRevolution/4*Math.PI);


    public float AngularMaxCorrection               =100;
    public float AngularMinCorrection               =15;
    public float LinearMaxCorrection                =100;
    public float LinearMinCorrection                =15;

    public float Linearlasterror=0;


    private int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    public enum DriveSyle{LinearWithEnocders, ClockwiseWithGyroScope,CounterClockwiseWithGyroScope}



    public String Dim                               = "DeviceInterfaceModule1";

    public double PidPowerAdjustment(int TargetAngle) {

        float LinearCumulativeerror = 0;
        float LinearproportionalCorrection;
        float LinearintegralCorrection;
        float LinearSlopeofderivitive;
        float LinearMaxCorrection = 100;
        float LinearMinCorrection = 15;
        float Linearerror = Math.abs(TargetAngle - getIntegratedZValue());
        LinearproportionalCorrection = (LinearproportionalConstant * Linearerror);
        LinearCumulativeerror += Linearerror;
        LinearintegralCorrection = (LinearintegralConstant * LinearCumulativeerror);
        LinearSlopeofderivitive = Linearerror - Linearlasterror;
        float Linearderivitivecorrection = (LinearSlopeofderivitive * LinearderivitiveConstant);


        float LinearCorrection = LinearproportionalCorrection + LinearintegralCorrection + Linearderivitivecorrection;

        if (LinearCorrection > LinearMaxCorrection) {
            LinearCorrection = LinearMaxCorrection;
        } else if (LinearCorrection < LinearMinCorrection) {
            LinearCorrection = LinearMinCorrection;
        } else LinearCorrection = LinearCorrection;
        return LinearCorrection;

    }

    public void initializeAllMotors(){}

    public void StopAllMotors(){}

    public void initializeAllServos(){}

    public void bringAllServosHome(){}

   /* String VuforiaLicenseKey = "AbkJpf//////AAAAGfwmmKkkGUDwrRcXe4puyLQhZ3m1wmsmuJUw2GVDtb7tWinUTnSd+UmyGz5aylC8ShWX8ayvA9h2mDtWnM1s3yni7S/WtH8buZO7gUBz9FotxNPJGL8Di9VJSmOhzEoyHLivQpx/vPwoH0Aejcvr1lBt8b5yMEgegLQ+WbmwNmj25ciaaMFDhryp7CTOzZFswvIUdhZ84PBJJew94ewMFjrsGNqra+0beno8wvEH9XmHp2kj9lVT+u8EjZdSQuEowkS5Lw2bnmOCMfPk9/00KZ+xBfaa2LDB3IXuYR2FVdd6qORTWXA8N120mYbCx8x8U7R4JdZs/eAH279CtHqFyFPdQtj3qn3Of7Z3urbcezNu";

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
        RobotTheta = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;


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
    }
*/


    }



