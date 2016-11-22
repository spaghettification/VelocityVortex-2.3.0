package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 11/21/2016.
 */
public class BlueCloseButtonPusher extends LinearHardwareMap {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousHardwareMap();
        StopAllMotors();
        InitializeServoPositions();
        Calibrate();
        waitForStart();
        Drive(.25, 8, 0, 5, false);
        //Possibly pause for a second to shoot Ball
        Turn(.125, 30, false);
        Drive(.25, 50, 0, 4, false);
        Turn(.125, 0, false);
        ButtonPush("Blue");
        setPower(0, 0, 0, 0);
        Turn(.125,0,false);
        Drive(.25,18,getIntegratedZValue(),5,false);
        ButtonPush("Blue");
        sleep(500);
        Drive(.125, -6, 90, 3, false);
        Turn(.125, 0, false);
        sleep(500);
        Drive(.125, -8, getIntegratedZValue(), 5, false);
        Turn(.125, 150, false);
        Drive(.25,50,getIntegratedZValue(),5,false);
        //Possibly Shoot Ball


    }
}

