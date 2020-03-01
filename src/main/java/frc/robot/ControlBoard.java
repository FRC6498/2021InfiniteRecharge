package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.lib.util.DriveSignal;

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

 
    public double getThrottle() {
        driverArcadeDrive();
        return throttle;
    }
    

   
    public double getTurn() {
        driverArcadeDrive();
        return turn;
    }

    boolean driveReduction=false;
    double driveReductionAmount = .7; //remember a higher number means less reduction
    
    
    
    double throttle=0;
    double turn=0;
    
    public void driverArcadeDrive() {
        throttle=0;
        turn=mDriver.getX(Hand.kLeft)*Constants.regularTurnReduction;

        if(Math.abs(turn)<.05) turn=0;
        
        

        if(mDriver.getTriggerAxis(Hand.kRight)>.05) {
            throttle=mDriver.getTriggerAxis(Hand.kRight);			
        }else if(mDriver.getTriggerAxis(Hand.kLeft)>.05) {
            throttle=-mDriver.getTriggerAxis(Hand.kLeft);
            turn=-turn;
        }else {
            throttle=0;
            turn=turn*Constants.kDriveSwivelReduction;
        }
        if(getDriveInverted()&&throttle!=0){
            //turn=turn;
            throttle=-throttle;
        }
        

        //System.out.println("turn: "+turn+" throttle: "+throttle);
    }

   
    public DriveSignal getDriveSignal() {
        boolean squareInputs=true;
        double xSpeed;
        double zRotation;
        
        driverArcadeDrive();
        

        xSpeed=throttle;
    
        zRotation = turn;
        
      
    
        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }
        
        

        double leftMotorOutput;
        double rightMotorOutput;
    
        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    
        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
            } else {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
            } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
            }
        }
        double m_rightSideInvertMultiplier = 1.0;

        if(throttle>.05) m_rightSideInvertMultiplier*=1.1;//larger than 1 for drift right.9; //bruh this is good
         else if(throttle<.05) m_rightSideInvertMultiplier*=1.1;//.9; //dont touch, or i'm a beat ya

        leftMotorOutput=(limit(leftMotorOutput) * 1);
        rightMotorOutput=(limit(rightMotorOutput) * 1 * m_rightSideInvertMultiplier);

        
    // System.out.println("Rot:"+turn+" xSpeed: "+xSpeed+" Left: "+leftMotorOutput+ " right: "+rightMotorOutput);
        return new DriveSignal(leftMotorOutput,rightMotorOutput,false);
        
    }

    boolean driveInverted=true;
 
    public boolean getDriveInverted() {
        if(mDriver.getStickButtonReleased(Hand.kLeft)){
            driveInverted=!driveInverted;
         //   if(driveInverted)CameraVision.setStreamMode(StreamMode.LimeMain);
           // else CameraVision.setStreamMode(StreamMode.USBMain);
        }
    
    
        return driveInverted;


    }

    protected double limit(double value) {
        if (value > 1.0) {
        return 1.0;
        }
        if (value < -1.0) {
        return -1.0;
        }
        return value;
    }

    public boolean getTractionControl(){
        return mDriver.getBButtonPressed();
    }
    
   public boolean getLowGear(){
       return mDriver.getBumper(Hand.kRight);
   }

   public boolean getHighGear(){
       return mDriver.getBumper(Hand.kLeft);
   }

   public boolean getIntake(){
       return mDriver.getPOV()==180;
   }
   public boolean getStopIntake(){
       return mDriver.getPOV()==0;
   }

   public boolean getPlow(){
       return mDriver.getPOV()==90;
   }
//CLIMBING------------------
   public boolean getStartClimb(){
       return mDriver.getXButtonPressed();
   }

   public boolean getStopClimb(){
       return mDriver.getYButtonPressed();
   }

   public double getLiftJog(){
       return -mDriver.getY(Hand.kLeft);
   }

   public double getWinchJog(){
       return -mDriver.getY(Hand.kRight);
   }

   public double getBalanceJog(){
       return mDriver.getX(Hand.kLeft);
   }



   //OPERATOR CONTROLS

   public double getTurretManual() {
        double val= -mOperator.getX(Hand.kRight);
        return (Math.abs(val) > Math.abs(.05)) ? val : 0.0;
    }

    public double getHoodTuningAdjustment(){
        double val= -mOperator.getY(Hand.kRight);
        return (Math.abs(val) > Math.abs(.05)) ? val : 0.0;
    }


    public boolean getAutoAim() {
        return mOperator.getBumperPressed(Hand.kRight);
    }

    public boolean getStopShooter(){
        return mOperator.getBumperPressed(Hand.kLeft);
    }

    public boolean getShooterOpenLoop() {
        return mOperator.getStickButtonPressed(Hand.kRight);
    }

    public boolean getShooterDumpToTrench(){
        return mOperator.getXButtonPressed();
    }

    public boolean getShooterFireOneWhenReady(){
        return mOperator.getTriggerAxis(Hand.kRight)>.5;
    }

    public boolean getShooterFireOneNow(){
        return mOperator.getTriggerAxis(Hand.kLeft)>.5;
    }

    public boolean getAutoShootOn(){
        return mOperator.getAButtonPressed();
    }

    public boolean getAutoShootoff(){
        return mOperator.getBButtonPressed();
    }

    boolean lastReadingAddBeltBall = false;
    public boolean addBeltBall(){
        boolean currentReading = mOperator.getPOV()==90;
        boolean result = buttonPressed(lastReadingAddBeltBall, currentReading);
        lastReadingAddBeltBall = currentReading;
        return result;
    }

    boolean lastReadingSubtractBeltBall = false;
    public boolean subtractBeltBall(){
        boolean currentReading = mOperator.getPOV()==270;
        boolean result = buttonPressed(lastReadingSubtractBeltBall, currentReading);
        lastReadingSubtractBeltBall = currentReading;
        return result;
    }

    boolean lastReadingAddFeederBall = false;
    public boolean addFeederBall(){
        boolean currentReading = mOperator.getPOV()==0;
        boolean result = buttonPressed(lastReadingAddFeederBall, currentReading);
        lastReadingAddFeederBall = currentReading;
        return result;
    }

    boolean lastReadingSubtractFeederBall = false;
    public boolean subtractFeederBall(){
        boolean currentReading = mOperator.getPOV()==180;
        boolean result = buttonPressed(lastReadingSubtractFeederBall, currentReading);
        lastReadingSubtractFeederBall = currentReading;
        return result;
    }


    private boolean buttonPressed(boolean lastReading, boolean currentReading){
        return lastReading==false && currentReading==true;
    }

    public boolean fillBalls(){
        return mOperator.getStickButtonPressed(Hand.kLeft);
    }

    public boolean addCSVValue(){
        return mOperator.getStartButtonPressed();
    }

  

    //RUMBLE ------------------------------------------------------------------------------
    public enum Controller {Driver,Operator}
    public enum RumbleSide {left, right, both}
 
    public void setRumble(Controller c, RumbleSide type, double amount) {
        XboxController controller;
        if(c==Controller.Driver) controller=mDriver;
        else controller = mOperator;

        switch(type){
            case left:
                controller.setRumble(RumbleType.kLeftRumble,amount);              
            break;
            case right:
               controller.setRumble(RumbleType.kRightRumble,amount);
            break;
            case both: 
               controller.setRumble(RumbleType.kLeftRumble, amount);
               controller.setRumble(RumbleType.kRightRumble,amount);
            break;
        }
        
    }

   
    public void rumbleOff() {
        setRumble(Controller.Driver, RumbleSide.both, 0);
        setRumble(Controller.Operator, RumbleSide.both, 0);
    }

    
    public void setRumble(double amount) {
        setRumble(Controller.Driver, RumbleSide.both, amount);
        setRumble(Controller.Operator, RumbleSide.both, amount);
    }


public double getTestControl(){
return mDriver.getY(Hand.kRight);
}
    
}
