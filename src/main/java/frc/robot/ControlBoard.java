package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * A basic framework for the control board Like the drive code, one instance of
 * the ControlBoard object is created upon startup, then other methods request
 * the singleton ControlBoard instance.
 */
public class ControlBoard {
    private static ControlBoard mInstance = new ControlBoard();

    public static ControlBoard getInstance() {
        return mInstance;
    }

  //  private final Joystick mThrottleStick;
  //  private final Joystick mTurnStick;
    private final XboxController mDriver, mOperator;

    private ControlBoard() {
      //  mThrottleStick = new Joystick(0);
      //  mTurnStick = new Joystick(1);
       mDriver = new XboxController(0);
       mOperator = new XboxController(1);

    }

    // DRIVER CONTROLS
    public double getThrottle() {
        double throttle=0;
        
        if(mDriver.getTriggerAxis(Hand.kRight)>.05) {
            throttle=mDriver.getTriggerAxis(Hand.kRight);			
        }else if(mDriver.getTriggerAxis(Hand.kLeft)>.05) {
            throttle=-mDriver.getTriggerAxis(Hand.kLeft);
           
        }else {
            throttle=0;
        }
       // if(getDriveInverted()&&throttle!=0){
            //turn=turn;
       //     throttle=-throttle;
       // }
       return throttle;
    }

    public double getTurn() {
        return mDriver.getX(Hand.kLeft);
    }

    public boolean getQuickTurn() {
        return mDriver.getAButton();
    }

    public boolean getTractionControl() {
        return mDriver.getBButton();
    }

   public boolean getAlignWithLoadStation(){
       return mDriver.getXButton();
   }
    
   public boolean getLowGear(){
       return mDriver.getBumper(Hand.kRight);
   }

   //OPERATOR CONTROLS

   public double getTurretManual() {
    if (mOperator.getXButton()) {
        return 1.0;
    } else if (mOperator.getYButton()) {
        return -1.0;
    } else {
        return 0.0;
    }
}

    public boolean getAutoAimNewBalls() {
        return mOperator.getAButton();
    }

    public boolean getAutoAimOldBalls() {
        return mOperator.getBButton();
    }

    public boolean getHoodTuningPositiveButton() {
        return mOperator.getPOV()==180;
    }

    public boolean getHoodTuningNegativeButton() {
        return mOperator.getPOV()==0;
    }
    


public double getTestControl(){
return mDriver.getY(Hand.kRight);
}
    
}
