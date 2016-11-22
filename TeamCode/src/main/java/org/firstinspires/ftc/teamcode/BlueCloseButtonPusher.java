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
        shootParticle(1);
        Turn(.125, 51, false);
        Drive(.25, 66.843, 0, 4, false);
        Turn(.125, 90, false);
        ButtonPush("Blue");
        setPower(0, 0, 0, 0);
        Drive(.125,-8)
        Turn(.125,0,false);
        Drive(.25,48,getIntegratedZValue(),5,false);
        Turn(.125,90,false);
        ButtonPush("Blue");
        sleep(500);
        Drive(.125, -6, 90, 3, false);
        Turn(.125, 0, false);
        sleep(500);
        Drive(.125, -8, getIntegratedZValue(), 5, false);
        Turn(.125, 150, false);
        Drive(.25,50,getIntegratedZValue(),5,false);
        shootParticle(1);


    }
    public void shootParticle(int Quantity){
    for (int i; i<Quantity+1;i++){
        while (!CatapultStop.ispressed){
        Catapult.setPower(1);
        }Catapult.setPower(0);
        sleep(300);
        while (CatapultStop.ispressed())Catapult.setPower(1);}Catapult.setPower(0);
        if (i>1){BallCollection.setPower(1){
               sleep (300);
               while (!CatapultStop.ispressed()){
                   Catapult.setPower(1);
               }Catapult.setPower(0)
              BallControl.setPosition(ballControlEngagedPosition);
                        sleep(2000);
                        
        
}
    }
}

