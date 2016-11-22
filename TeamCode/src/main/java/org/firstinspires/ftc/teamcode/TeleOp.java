package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 10/2/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop", group = "6994 Bot")
public class TeleOp extends FTC_6994_Template {
    @Override
    public void init() {
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        CapBallLiftLeft=hardwareMap.dcMotor.get(capBallLiftLeft);
        CapBallLiftRight=hardwareMap.dcMotor.get(capBallLiftRight);
        CapBallLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        CapBallFork=hardwareMap.servo.get(capBallFork);
        CapBallarm1=hardwareMap.crservo.get(capBallarm1);
        CapBallarm2=hardwareMap.crservo.get(capBallarm2);*/
        BallControl=hardwareMap.servo.get(ballControll);
        BallCollection = hardwareMap.dcMotor.get(ballCollection);
        Gyro = hardwareMap.gyroSensor.get(gyroSensor);
        Catapult = hardwareMap.dcMotor.get(catapult);
        CatapultStop = hardwareMap.touchSensor.get(catapultStop);


    }

    public void start() {
        mRuntime.reset();
    }

    @Override
    public void loop() {
        BallCollection.setPower(-scaleInput(gamepad2.right_stick_y));
        float LeftPower = gamepad1.right_stick_y;
        float RightPower = -gamepad1.left_stick_y;
            if (gamepad1.left_bumper){
                FrontLeft.setPower(Range.clip(scaleInput(LeftPower)/2,-.5,.5));
                FrontRight.setPower(Range.clip(scaleInput(RightPower)/2,-.5,.5));
                BackLeft.setPower(Range.clip(scaleInput(LeftPower)/2,-.5,.5));
                BackRight.setPower(Range.clip(scaleInput(RightPower)/2,-.5,.5));}
            else{
                FrontLeft.setPower(scaleInput(LeftPower));
                FrontRight.setPower(scaleInput(RightPower));
                BackLeft.setPower(scaleInput(LeftPower));
                BackRight.setPower(scaleInput(RightPower));}

            if (CatapultStop.isPressed()){
                if (gamepad2.dpad_up){Catapult.setPower(1);}
                else {Catapult.setPower(0);}}

            else{
                if (gamepad2.dpad_down){Catapult.setPower(1);}
            else {Catapult.setPower(0);}
            }
        /*if (gamepad2.left_bumper){
            Catapult.setPower(gamepad2.right_stick_y);
            BallCollection.setPower(scaleInput(gamepad2.right_stick_y));
        }*/


            if (gamepad1.left_bumper){BallControl.setPosition(ballControlEngagedPosition);}

            if (gamepad1.right_bumper){BallControl.setPosition(ballControlStartPosition);}

    }

    public void stop() {
        setDrivePower(0, 0, 0, 0);
    }

    enum Color{Red,Blue,Null}

}
