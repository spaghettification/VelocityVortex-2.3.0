package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 9/17/2016.
 */
public class BlueAAutonomous extends FTC_6994_Template {
    Servo Button;
    public double DistanceFromWall = Rangesensor.getDistance(DistanceUnit.INCH);
    public int Theta = (int) CalculateTheta(DistanceFromWall);
    enum State {Drive_to_Far_Beacon, Drive_to_Close_Beacon,Press_Far_Button,Press_Close_Button, Drive_To_Vortex, Prime_For_Launch, Launch, Return}
    
    private void newState(State newState) {
        mStateTime.reset();
        State mCurrentState = newState;
    }
    private final PathSeg[] FarBeaconPath = {
            new PathSeg(.4,0,4,DriveStyle.Linear),
    };
    private final PathSeg[] CloseBeaconPath = {
            new PathSeg(.125,0,0,DriveStyle.SquareWithBack),
            new PathSeg(.25, 0, HTolerance-((ODStoInch(BackLeftODS)+ODStoInch(BackRightODS))/2), DriveStyle.Linear),
            new PathSeg(.375, Theta, 0, DriveStyle.Clockwise),
            new PathSeg(.5, 0, CalculateHypotenuse(DistanceFromWall), DriveStyle.Linear),
            new PathSeg(.2, -Theta, 0, DriveStyle.CounterClockwise),
            new PathSeg(.125,0,0,DriveStyle.SquareWithSide),
            new PathSeg(.25, 0, 0, DriveStyle.FindWhiteLine),

    };
    private final PathSeg[] ToVortex = {
            new PathSeg(.4,0,4,DriveStyle.Linear),
    };

    public void loop(){

    }

    public void State(){
        State mCurrentState;
        mCurrentState=State.Drive_to_Far_Beacon;
        switch (mCurrentState){
            case Drive_to_Far_Beacon:{
                startPath(FarBeaconPath);
                if(pathComplete()){newState(State.Press_Far_Button);
                }
            }
            break;
            case Press_Far_Button:{
                double CurrentTime=SystemClock.currentThreadTimeMillis();
                PushButton(TeamColor.Blue,Button);
                if (SystemClock.currentThreadTimeMillis()-CurrentTime>500){newState(State.Drive_to_Close_Beacon);
                }
            }
            break;
            case Drive_to_Close_Beacon:{startPath(CloseBeaconPath);
                if (pathComplete()) {
                    newState(State.Press_Close_Button);
                }}
            break;
            case Press_Close_Button:{
                PressBeaconButton();
                newState(State.Drive_To_Vortex);
            }
            break;
            case Drive_To_Vortex:{startPath(ToVortex);}
            if(pathComplete()){newState(State.Prime_For_Launch);}
            break;
            case Prime_For_Launch:{}
            break;
            case Launch:{}
            break;
            case Return:{}
            break;

        }}
        public void PressBeaconButton(){
        if (BeaconCS.blue()>BeaconCS.red()&&BeaconCS.blue()>BeaconCS.green()){
            ButtonPusherLeft.setPosition(.3);
            ButtonPusherLeft.setPosition(0);
        }
            else{
            ButtonPusherRight.setPosition(.8);
            ButtonPusherRight.setPosition(1);

        }

    }
}
