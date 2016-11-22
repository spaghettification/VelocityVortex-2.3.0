package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.annotation.Target;
import java.util.Set;

/**
 * Created by Trevor on 11/6/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LongDistance", group = "6994 Bot")
public class LongDistanceBallShooter extends LinearHardwareMap {
    int InitialTheta = 30;
    double HypotenuseLength = 50;
    double HypotenuseDriveTime = 3;
    ElapsedTime runtime = new ElapsedTime();
    public float Linearlasterror;
    double LeftPower = 0;
    double RightPower = .25;
    double ReverseLeftPower = .25;
    double ReverseRightPower = 0;
    int TimeOut=3;
    double minPower=.25;
    double Runtime=3;


    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);
        Gyro = hardwareMap.gyroSensor.get(gyroSensor);
        ButtonPusherLeft = hardwareMap.servo.get(buttonPusherLeft);
        ButtonPusherRight = hardwareMap.servo.get(buttonPusherRight);
        BeaconColorSensor = hardwareMap.colorSensor.get(beaconColorSensor);
        WhiteLineFinder = hardwareMap.colorSensor.get(whiteLineFinder);
        BeaconColorSensor = hardwareMap.colorSensor.get(beaconColorSensor);
        Catapult = hardwareMap.dcMotor.get(catapult);
        CatapultStop = hardwareMap.touchSensor.get(catapultStop);
        BallCollection = hardwareMap.dcMotor.get(ballCollection);
        BallControl = hardwareMap.servo.get(ballControll);
        BallControl.setPosition(ballControlStartPosition);
        SideRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, sideRangeSensor);
        FrontRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, frontRangeSensor);
        BackRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, backRangeSensor);
//
        //FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        setPower(0,0,0,0);
        Gyro.calibrate();
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();

        SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(2000);


        /*while (Gyro.isCalibrating() && opModeIsActive()) {
            telemetry.addData(">", "Calibrating Gyro");
            telemetry.update();
            idle();
            sleep(50);
            telemetry.addData(">", "Ready!");
            telemetry.addData(">", "Hey Jason, Try not to Fuck up");
            telemetry.update();
        }*/

        waitForStart();

        if(opModeIsActive())
        {
            BallControl.setPosition(ballControlStartPosition);
            setPower(.25, .25, .25, .25);
            sleep(5050);
            setPower(.25,-.25,.25,-.25);
            sleep(800);
            Catapult.setPower(1);
            sleep(8000);
            while (!CatapultStop.isPressed()) {
                Catapult.setPower(1);
            }
            Catapult.setPower(0);
            sleep(2000);
            BallCollection.setPower(1);
            sleep(500);
            BallControl.setPosition(ballControlEngagedPosition);
            sleep(5000);
            BallControl.setPosition(ballControlStartPosition);
            sleep(500);
            Catapult.setPower(1);
            BallCollection.setPower(0);
            sleep(4000);
            Catapult.setPower(0);
            setPower(.5, .5, .25, .25);
            sleep(8000);
            setPower(0, 0, 0, 0);
            setPower(0, 0, 0, 0);
            Catapult.setPower(0);
            BallCollection.setPower(0);

        }
        else
        {
            BallControl.setPosition(ballControlStartPosition);
            BallCollection.setPower(0);
            setPower(0,0,0,0);
            Catapult.setPower(0);

        }

/*

                if (gamepad1.a && gamepad1.right_bumper){
                    LeftPower+=.05;
                }
                if (gamepad1.b && gamepad1.right_bumper){
                    LeftPower-=.05;
                }
                if (gamepad1.x && gamepad1.right_bumper){
                    RightPower+=.05;
                }
                if (gamepad1.y && gamepad1.right_bumper){
                    RightPower-=.05;
                }
                if (gamepad1.dpad_up && gamepad1.right_bumper){
                    ReverseLeftPower+=.05;
                }
                if (gamepad1.dpad_down && gamepad1.right_bumper){
                    ReverseLeftPower-=.05;
                }
                if (gamepad1.dpad_left && gamepad1.right_bumper){
                    ReverseRightPower+=.05;
                }
                if (gamepad1.dpad_right && gamepad1.right_bumper){
                    ReverseRightPower-=.05;
                }

                if (gamepad2.a && gamepad2.right_bumper){
                    minPower+=.05;
                }
                if (gamepad2.b && gamepad2.right_bumper){
                    minPower-=.05;
                }
                if (gamepad2.x && gamepad2.right_bumper){
                    Runtime+=.125;
                }
                if (gamepad2.y && gamepad2.right_bumper){
                    Runtime-=.125;
                }
                if (gamepad2.dpad_up && gamepad2.right_bumper){
                    TimeOut+=.05;
                }
                if (gamepad2.dpad_down && gamepad2.right_bumper){
                    TimeOut-=.05;
                }




                if (gamepad1.a) {
                    TurnWithoutEncoder(LeftPower, RightPower, 45, TimeOut);
                }
                else if (gamepad1.b){
                    TurnWithoutEncoder(LeftPower,RightPower,50,TimeOut);
                }
                else if (gamepad1.x){
                    TurnWithoutEncoder(LeftPower,RightPower,55,TimeOut);
                }
                else if (gamepad1.y){
                    TurnWithoutEncoder(LeftPower,RightPower,60,TimeOut);
                }
                else if (gamepad1.dpad_down){
                    TurnWithoutEncoder(LeftPower,RightPower,40,TimeOut);
                }
                else if (gamepad1.dpad_up){
                    TurnWithoutEncoder(LeftPower,RightPower,35,TimeOut);
                }
                else if (gamepad1.dpad_left){
                    TurnWithoutEncoder(LeftPower,RightPower,30,TimeOut);
                }
                else if (gamepad1.dpad_right){
                    TurnWithoutEncoder(LeftPower,RightPower,25,TimeOut);
                }
                else if (gamepad1.a && gamepad1.left_bumper){
                    TurnWithoutEncoder(ReverseLeftPower,ReverseRightPower,-45,TimeOut);
                }
                else if (gamepad1.b && gamepad1.left_bumper){
                    TurnWithoutEncoder(ReverseLeftPower,ReverseRightPower,-50,TimeOut);
                }
                else if (gamepad1.x && gamepad1.left_bumper){
                    TurnWithoutEncoder(ReverseLeftPower,ReverseRightPower,-55,TimeOut);
                }
                else if (gamepad1.y && gamepad1.left_bumper){
                    TurnWithoutEncoder(ReverseLeftPower,ReverseRightPower,-60,TimeOut);
                }
                else if (gamepad1.dpad_left && gamepad1.left_bumper){
                    TurnWithoutEncoder(ReverseLeftPower,ReverseRightPower,-40,TimeOut);
                }
                else if (gamepad1.dpad_right && gamepad1.left_bumper){
                    TurnWithoutEncoder(ReverseLeftPower,ReverseRightPower,-35,TimeOut);
                }
                else if (gamepad1.dpad_up && gamepad1.left_bumper){
                    TurnWithoutEncoder(ReverseLeftPower,ReverseRightPower,-30,TimeOut);
                }
                else if (gamepad1.dpad_down && gamepad1.left_bumper){
                    TurnWithoutEncoder(ReverseLeftPower,ReverseRightPower,-25,TimeOut);
                }
                else {setPower(0,0,0,0);}

                if (gamepad2.a){
                    DriveWithoutEncoder(minPower,getIntegratedZValue(),runtime.seconds()<2,false);
                }
                else if (gamepad2.b){
                    DriveWithoutEncoder(minPower,getIntegratedZValue(),runtime.seconds()<2.5,false);
                }
                else if (gamepad2.x){
                    DriveWithoutEncoder(minPower,getIntegratedZValue(),runtime.seconds()<3,false);
                }
                else if (gamepad2.y){
                    DriveWithoutEncoder(minPower,getIntegratedZValue(),runtime.seconds()<3.5,false);
                }
                else if (gamepad2.dpad_down){
                    DriveWithoutEncoder(minPower,getIntegratedZValue(),runtime.seconds()<4,false);
                }
                else if (gamepad2.dpad_up){
                    DriveWithoutEncoder(minPower,getIntegratedZValue(),runtime.seconds()<4.5,false);
                }
                else if (gamepad2.dpad_left){
                    DriveWithoutEncoder(minPower,getIntegratedZValue(),runtime.seconds()<5,false);
                }
                else if (gamepad2.dpad_right){
                    DriveWithoutEncoder(minPower,getIntegratedZValue(),runtime.seconds()<5.5,false);
                }
                if (gamepad1.start){
                    TurnWithoutEncoder(LeftPower,RightPower,getIntegratedZValue()+5,5);
                }
                if (gamepad1.back){
                    TurnWithoutEncoder(LeftPower,RightPower,getIntegratedZValue()-5,5);
                }
                else{setPower(0,0,0,0);}
                telemetry.addData("Heading", getIntegratedZValue());
                telemetry.addData("LeftPower", LeftPower);
                telemetry.addData("RightPower",RightPower);
                telemetry.addData("ReverseRightPower",ReverseRightPower);
                telemetry.addData("ReverseLeftPower",ReverseRightPower);
                telemetry.addData("minPower",minPower);
                telemetry.update();
*/

    }


        /*
            runtime.reset();
            sleep(50);
            DriveWithoutEncoder(.375, InitialTheta, runtime.seconds() < HypotenuseDriveTime, false);
            runtime.reset();
            sleep(50);
            TurnWithoutEncoder(.25, 0, 0,3);
            runtime.reset();
            sleep(50);
            DriveWithoutEncoder(.375, 0, !WhiteLineFound(), false);
            runtime.reset();
            sleep(50);
            sleep(50);
            runtime.reset();
    */


    public boolean WhiteLineFound() {

        return ((WhiteLineFinder.blue() > 180 && WhiteLineFinder.red() > 180 && WhiteLineFinder.green() > 180));
    }

    public void TurnWithoutEncoder(double LeftPower, double RightPower, int TargetAngle,int Timeout) {

        while (runtime.seconds()<Timeout){
            if (Math.abs(getIntegratedZValue() - TargetAngle) > 1.25) {
                if (getIntegratedZValue() > TargetAngle) {
                    setPower(LeftPower, RightPower, LeftPower, RightPower);
                } else if (getIntegratedZValue() < TargetAngle) {
                    setPower(LeftPower, RightPower, LeftPower, RightPower);
                } else {
                    setPower(LeftPower, RightPower, LeftPower, RightPower);
                }
            } else {
                setPower(0, 0, 0, 0);
            }
            telemetry.addData(">","Turning");
            telemetry.addData("LeftPower", LeftPower);
            telemetry.addData("RightPower", RightPower);
            telemetry.addData("Gyro Heading", getIntegratedZValue());
            telemetry.addData("TargetAngle",TargetAngle);
            telemetry.addData("TimeOut", Timeout);
            telemetry.addData("RunTime", runtime.seconds());
            telemetry.update();
        }

        sleep(50);
    }

    public void DriveWithoutEncoder(double minPower, int AngleToMaintain, boolean StoppingEvent, boolean PIDdesired) {
        double FrontLeftDynamicPower;
        double FrontRightDynamicPower;
        double BackLeftDynamicPower;
        double BackRightDynamicPower;
        SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(100);
        while (StoppingEvent) {
            if (!StoppingEvent) {
                if (getIntegratedZValue() > AngleToMaintain && PIDdesired) {//VeeringRight

                    FrontLeftDynamicPower = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
                    FrontRightDynamicPower = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
                    BackLeftDynamicPower = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
                    BackRightDynamicPower = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);

                } else if (getIntegratedZValue() < AngleToMaintain && PIDdesired) {//VeeringLeft

                    FrontLeftDynamicPower = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
                    FrontRightDynamicPower = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
                    BackLeftDynamicPower = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
                    BackRightDynamicPower = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
                } else {

                    FrontLeftDynamicPower = Range.clip(minPower, 0, 1);
                    FrontRightDynamicPower = Range.clip(minPower, 0, 1);
                    BackLeftDynamicPower = Range.clip(minPower, 0, 1);
                    BackRightDynamicPower = Range.clip(minPower, 0, 1);
                }
                setPower(FrontLeftDynamicPower, FrontRightDynamicPower, BackLeftDynamicPower, BackRightDynamicPower);
            }
            else setPower(0,0,0,0);
            telemetry.addData(">","Driving");
            telemetry.addData("minPower", minPower);
            telemetry.addData("AngleToMaintain", AngleToMaintain);
            telemetry.addData("RunTime", runtime.seconds());
            telemetry.update();
        }
        setPower(0, 0, 0, 0);
        sleep(50);
    }

    public void Drive(double minPower, int Distance, int TargetAngle, double TimeOut, boolean PIDdesired) {
        double FrontLeftDynamicPower;
        double FrontRightDynamicPower;
        double BackLeftDynamicPower;
        double BackRightDynamicPower;
        int AngleToMaintain = getIntegratedZValue();

        sleep(200);
        //SetMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        sleep(300);
    }

    public void Turn(double Power, int TargetAngle) {
        double FrontLeftTurnPower = 0;
        double FrontRightTurnPower = 0;
        double BackLeftTurnPower = 0;
        double BackRightTurnPower = 0;
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(300);
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
    }

    public void pressButton(String TeamColor, double LeftButtonPusherEngagedPosition, double RightButtonPusherEngagedPosition, double LeftButtonPusherStartPosition, double RightButtonPusherStartPosition) {
        if (BeaconColorSensor.blue() > BeaconColorSensor.red() && BeaconColorSensor.blue() > BeaconColorSensor.red()) {
            switch (TeamColor.toLowerCase()) {
                case "blue": {
                    if (BeaconColorSensor.blue() > BeaconColorSensor.red() && BeaconColorSensor.blue() > BeaconColorSensor.green()) {
                        setPower(-.125,-.125,-.125,-.125);
                        sleep(600);
                        setPower(.125,-.125,.125,-.125);
                    } else
                        setPower(-.125,-.125,-.125,-.125);
                    sleep(1200);
                    setPower(.125,-.125,.125,-.125);
                }
                break;
                case "red": {
                    if (BeaconColorSensor.blue() > BeaconColorSensor.red() && BeaconColorSensor.blue() > BeaconColorSensor.green()) {
                        setPower(-.125,-.125,-.125,-.125);
                        sleep(600);
                        setPower(.125,-.125,.125,-.125);
                    } else
                        setPower(-.125,-.125,-.125,-.125);
                    sleep(1200);
                    setPower(.125,-.125,.125,-.125);
                }
                break;
            }
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

    public void SetPowerwithPIDAdjustment(double minPower, int TargetAngle, boolean PIDdesired) {
        double FrontLeftDynamicPower;
        double FrontRightDynamicPower;
        double BackLeftDynamicPower;
        double BackRightDynamicPower;
        int AngleToMaintain = TargetAngle;
        if (getIntegratedZValue() > TargetAngle && PIDdesired) {//VeeringRight

            FrontLeftDynamicPower = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
            FrontRightDynamicPower = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
            BackLeftDynamicPower = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
            BackRightDynamicPower = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);

        } else if (getIntegratedZValue() < TargetAngle && PIDdesired) {//VeeringLeft

            FrontLeftDynamicPower = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
            FrontRightDynamicPower = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
            BackLeftDynamicPower = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
            BackRightDynamicPower = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
        } else {

            FrontLeftDynamicPower = Range.clip(minPower, 0, 1);
            FrontRightDynamicPower = Range.clip(minPower, 0, 1);
            BackLeftDynamicPower = Range.clip(minPower, 0, 1);
            BackRightDynamicPower = Range.clip(minPower, 0, 1);
        }

        setPower(FrontLeftDynamicPower, FrontRightDynamicPower, BackLeftDynamicPower, BackRightDynamicPower);
    }

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

    public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

}

