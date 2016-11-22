package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 10/29/2016.
 */
public class BlueAuto6994 extends LinearOpMode {
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Catapult;

    GyroSensor Gyro;

    ModernRoboticsI2cRangeSensor RightSideRange;
    ModernRoboticsI2cRangeSensor BackLeftRange;
    ModernRoboticsI2cRangeSensor BackRightRange;

    ColorSensor ButtonDetection;
    ColorSensor WhiteLineFinder;

    TouchSensor CatapultStop;

    Servo ButtonPusherLeft;
    Servo ButtonPusherRight;

    int Initial_Pushoff=6;
    int RightSideBuffer=4;
    int Distance_From_Back_Wall_To_White_Line=36;



    private ElapsedTime runtime=new ElapsedTime();
    private double AndyMarkMotor_TicksPerRevolution=1120;
    private double CountsPerInch=(AndyMarkMotor_TicksPerRevolution/4*Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();

        Gyro = hardwareMap.gyroSensor.get("G");
        Gyro.calibrate();

        RightSideRange= hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RSR");
        BackLeftRange= hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "BLR");
        BackRightRange= hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "BRR");
        double Distance_From_Right_Side_Wall=RightSideRange.getDistance(DistanceUnit.INCH);
        double Hypotenuse_To_White_Line=Math.sqrt(((Distance_From_Right_Side_Wall-RightSideBuffer)*(Distance_From_Right_Side_Wall-RightSideBuffer))+((Distance_From_Back_Wall_To_White_Line-Initial_Pushoff)+(Distance_From_Back_Wall_To_White_Line-Initial_Pushoff)));
        int Theta_Of_Path=(int) Math.atan((Distance_From_Back_Wall_To_White_Line-Initial_Pushoff)/(Distance_From_Right_Side_Wall-RightSideBuffer));

        ButtonDetection=hardwareMap.colorSensor.get("bd");
        WhiteLineFinder=hardwareMap.colorSensor.get("wlf");

        CatapultStop=hardwareMap.touchSensor.get("cs");

        ButtonPusherLeft=hardwareMap.servo.get("bpl");
        ButtonPusherRight=hardwareMap.servo.get("bpr");
        ButtonPusherLeft.setPosition(0);
        ButtonPusherRight.setPosition(1);
        sleep(500);
        waitForStart();
        PressBeacons(Theta_Of_Path,Hypotenuse_To_White_Line);



    }
    public void initializeMotors(){
        FrontLeft=hardwareMap.dcMotor.get("FL");
        FrontRight=hardwareMap.dcMotor.get("FR");
        BackLeft=hardwareMap.dcMotor.get("BL");
        BackRight=hardwareMap.dcMotor.get("BR");


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(2000);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveWithEncoders(DriveSyle driveSyle, double Power,int TargetAngle, int MaxSpeed, double Distance, double Pause){
        int FrontLeftCurrent = FrontLeft.getCurrentPosition();
        int FrontRightCurrent = FrontRight.getCurrentPosition();
        int BackLeftCurrent = BackLeft.getCurrentPosition();
        int BackRightCurrent = BackRight.getCurrentPosition();
        switch (driveSyle){


            case LinearWithEnocders:{
                if (opModeIsActive()){
                    int FrontLeftTarget= (FrontLeftCurrent+(int)(Distance*CountsPerInch));
                    int FrontRightTarget= (FrontRightCurrent+(int)(Distance*CountsPerInch));
                    int BackLeftTarget= (BackLeftCurrent+(int)(Distance*CountsPerInch));
                    int BackRightTarget= (BackRightCurrent+(int)(Distance*CountsPerInch));

                    setMaxSpeed(MaxSpeed);
                    setTargetPosition(FrontLeftTarget,FrontRightTarget,BackLeftTarget,BackRightTarget);
                    setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    setPower(Power,Power,Power,Power);
                    while (  opModeIsActive()&& FrontLeft.isBusy() &&FrontRight.isBusy() &&BackLeft.isBusy() &&BackRight.isBusy() &&runtime.seconds()<Pause)
                    {
                        telemetry.addData("FrontLeft",FrontLeft.getCurrentPosition());
                        telemetry.addData("FrontRight",FrontRight.getCurrentPosition());
                        telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
                        telemetry.addData("BackRight",BackRight.getCurrentPosition());
                        telemetry.update();
                    }
                    setPower(0,0,0,0);
                    setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(200);
                    setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER );

                }
            }break;

            case ClockwiseWithGyroScope:{
                if (opModeIsActive()){

                    setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    runtime.reset();
                    while (Math.abs(getIntegratedZValue()-TargetAngle)<1.2&&runtime.seconds()>Pause){
                        setMaxSpeed(MaxSpeed);
                        setPower(Power,-Power,Power,-Power);
                    }setPower(0,0,0,0);
                }
            }break;

            case CounterClockwiseWithGyroScope:{
                if (opModeIsActive()){
                    setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    runtime.reset();
                    while (Math.abs(getIntegratedZValue()-TargetAngle)<1.2&&runtime.seconds()>Pause){
                        setMaxSpeed(MaxSpeed);
                        setPower(-Power,Pause,-Power,Power);
                    }
                    setPower(0,0,0,0);

                }
            }break;
            case FindWhiteLine:{
                setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while (!WhiteLineFinder.equals(16)&&runtime.seconds()<Pause){
                    setMaxSpeed(MaxSpeed);
                    setPower(Power,Power,Power,Power);
                }
                setPower(0,0,0,0);
            }break;
        }
    }
    public void setMaxSpeed(int MaxSpeed){
        FrontLeft.setMaxSpeed(MaxSpeed);
        FrontRight.setMaxSpeed(MaxSpeed);
        BackLeft.setMaxSpeed(MaxSpeed);
        BackRight.setMaxSpeed(MaxSpeed);
    }
    public void setPower(double FL,double FR,double BL,double BR){
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR);
        BackLeft.setPower(BL);
        BackRight.setPower(BR);
    }
    public void setTargetPosition(int FL,int FR,int BL, int BR){
        FrontLeft.setTargetPosition(FL);
        FrontRight.setTargetPosition(FR);
        BackLeft.setTargetPosition(BL);
        BackRight.setTargetPosition(BR);
    }
    public void pressButton(int pause){
        if (ButtonDetection.blue()>ButtonDetection.red()&&ButtonDetection.blue()>ButtonDetection.green()){
            moveServo(ButtonPusherLeft,.3,pause);
            sleep(1000);
        }
        else{moveServo(ButtonPusherRight,.8,pause);}
    }
    public void moveServo(Servo servo, double Position, int Timeout){
        double Start=servo.getPosition();
        runtime.reset();
        do {

            servo.setPosition(Position);
        } while ((servo.getPosition()>Position||servo.getPosition()<Position)&&runtime.seconds()<Timeout);{}
        servo.setPosition(Start);
    }
    public void setDriveMotorMode(DcMotor.RunMode Mode){
        FrontLeft.setMode(Mode);
        FrontRight.setMode(Mode);
        BackLeft.setMode(Mode);
        BackRight.setMode(Mode);
    }
    enum DriveSyle{LinearWithEnocders, ClockwiseWithGyroScope,CounterClockwiseWithGyroScope,FindWhiteLine}
    public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }
    public void PressBeacons(int Theta_Of_Path,double Hypotenuse_To_White_Line){
        sleep(500);
        DriveWithEncoders(DriveSyle.LinearWithEnocders,.5,0,3000,Initial_Pushoff,3);
        DriveWithEncoders(DriveSyle.ClockwiseWithGyroScope,.125,Theta_Of_Path,1500,0,3);
        DriveWithEncoders(DriveSyle.LinearWithEnocders,.375,Theta_Of_Path,2000,Hypotenuse_To_White_Line,5);
        DriveWithEncoders(DriveSyle.CounterClockwiseWithGyroScope,.375,0,1500,0,3);
        DriveWithEncoders(DriveSyle.FindWhiteLine,.125,0,1000,0,5);
        pressButton(2);
        DriveWithEncoders(DriveSyle.LinearWithEnocders,.125,0,1500,20,3);
        DriveWithEncoders(DriveSyle.FindWhiteLine,.125,0,1500,0,5);
        pressButton(2);}

}
