/*
  2022 everybot code
  written by carson graf 
  don't email me, @ me on discord
*/

/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.*;
//mport com.ctre.phoenix.motorcontrol.IMotorController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;

public class Robot extends TimedRobot {
  
  //Definitions for the hardware. Change this if you change what stuff you have plugged in
  VictorSPX driveLeftA = new VictorSPX(20);
  TalonSRX driveLeftB = new TalonSRX(21);
  VictorSPX driveRightA = new VictorSPX(10);
  TalonSRX driveRightB = new TalonSRX(11);
  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  TalonSRX intake = new TalonSRX(6);
  Encoder encoder = new Encoder(0, 1);

  Joystick driverController = new Joystick(0);

  //Constants for controlling the arm. consider tuning these for your particular robot
  final double armHoldUp = 0.08;
  final double armHoldDown = 0.13;
  final double armTravel = 0.5;
  final double armTimeUp = 0.55;
  final double armTimeDown = 0.35;

  //Varibles needed for the 
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  
  // unknown
  boolean burstMode = false;
  double lastBurstTime = 0;

  //Autonomous vars
  double autoStart = 0;
  boolean goForAuto = false;
  double numBallsAuto;

  //PID loops stuff
  AHRS ahrs;
  PIDController turnController;
  double rotateToAngleRate;

  // PID Constants
  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system. Note that the */
  /* SmartDashboard in Test mode has support for helping you tune */
  /* controllers by displaying a form where you can enter new P, I, */
  /* and D constants and test the mechanism. */
  static final double kP = 0.3;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Configure motors to turn correct direction. You may have to invert some of your motors
    driveLeftA.setInverted(true);
    // driveLeftA.burnFlash();
    driveLeftB.setInverted(true);
    // driveLeftB.burnFlash();
    driveRightA.setInverted(false);
    // driveRightA.burnFlash();
    driveRightB.setInverted(false);
    // driveRightB.burnFlash();
    // driveLeftB.configAllSettings();
    
    arm.setInverted(false);
    arm.setIdleMode(IdleMode.kBrake);
    arm.burnFlash();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putNumber("Num Balls Auto", 0);
    numBallsAuto = SmartDashboard.getNumber("Num Balls Auto", 0);
    //goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
    // CameraServer.startAutomaticCapture();

    // PID for turns in Auto
    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
       * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * VMX-pi: - Communication via USB. - See
       * https://vmx-pi.kauailabs.com/installation/roborio-installation/
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }


    turnController = new PIDController(kP, kI, kD);
    turnController.enableContinuousInput(-180.0f, 180.0f);

  }

  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    //goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
    numBallsAuto = SmartDashboard.getNumber("Num Balls Auto", 0);
    //Reset the Navx
    ahrs.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //arm control code. same as in teleop
    //Their AUTO
    if(armUp)
    {
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldUp);
      }
    }
    
    //get time since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    // if(false){
    //   //series of timed events making up the flow of auto
    //   if(autoTimeElapsed < 3){
    //     //spit out the ball for three seconds
    //     //intake.set(ControlMode.PercentOutput, -1);
    //   }else if(autoTimeElapsed < 6){
    //     //stop spitting out the ball and drive backwards *slowly* for three seconds
    //     // intake.set(ControlMode.PercentOutput, 0);
    //     driveLeftA.set(ControlMode.PercentOutput,-0.3);
    //     driveLeftB.set(ControlMode.PercentOutput,-0.3);
    //     driveRightA.set(ControlMode.PercentOutput,-0.3);
    //     driveRightB.set(ControlMode.PercentOutput,-0.3);
    //   }else{
    //     //do nothing for the rest of auto
    //     // intake.set(ControlMode.PercentOutput, 0);
    //     driveLeftA.set(ControlMode.PercentOutput,0);
    //     driveLeftB.set(ControlMode.PercentOutput,0);
    //     driveRightA.set(ControlMode.PercentOutput,0);
    //     driveRightB.set(ControlMode.PercentOutput,0);
    //   }
    // }

    //My Auto
    //double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    boolean flag = false;
    boolean flag2 = false;
    // double currentRotationRate = MathUtil.clamp(turnController.calculate(ahrs.getAngle()), -1.0, 1.0);
    switch((int)numBallsAuto)
    {
      case 0:
        // no balls, just taxi
        if(autoTimeElapsed < 2)
        {
          driveLeftA.set(ControlMode.PercentOutput,0.4);
          driveLeftB.set(ControlMode.PercentOutput,0.4);
          driveRightA.set(ControlMode.PercentOutput,0.4);
          driveRightB.set(ControlMode.PercentOutput,0.4);
        }
        else
        {
          driveLeftA.set(ControlMode.PercentOutput,0);
          driveLeftB.set(ControlMode.PercentOutput,0);
          driveRightA.set(ControlMode.PercentOutput,0);
          driveRightB.set(ControlMode.PercentOutput,0);
        }
        break;


      case 1:
        // One ball and taxi
        if(autoTimeElapsed < 2)
        {
          intake.set(ControlMode.PercentOutput, -1);
        }
        else if(autoTimeElapsed < 4)
        {
          driveLeftA.set(ControlMode.PercentOutput,0.4);
          driveLeftB.set(ControlMode.PercentOutput,0.4);
          driveRightA.set(ControlMode.PercentOutput,0.4);
          driveRightB.set(ControlMode.PercentOutput,0.4);
          intake.set(ControlMode.PercentOutput, 0);
        }
        else
        {
          driveLeftA.set(ControlMode.PercentOutput,0);
          driveLeftB.set(ControlMode.PercentOutput,0);
          driveRightA.set(ControlMode.PercentOutput,0);
          driveRightB.set(ControlMode.PercentOutput,0);
        }
        break;


      case 2:
        
        if(autoTimeElapsed < 1.5)
        {
          armUp = false;
          intake.set(ControlMode.PercentOutput, 1);
        }
        else if (autoTimeElapsed < 2)
        {
          driveLeftA.set(ControlMode.PercentOutput,-0.4);
          driveLeftB.set(ControlMode.PercentOutput,-0.4);
          driveRightA.set(ControlMode.PercentOutput,-0.4);
          driveRightB.set(ControlMode.PercentOutput,-0.4);
        }
        else if(autoTimeElapsed < 5)
        {
          intake.set(ControlMode.PercentOutput, 0);
          armUp = true;

          // flag = false;
          flag2 = true;
          //double currentRotationRate = MathUtil.clamp(turnController.calculate(ahrs.getAngle()), -1.0, 1.0);
        }
        if(flag2)
        {

          while (ahrs.getAngle() < 180-20)
          {
              driveLeftA.set(ControlMode.PercentOutput,-0.4);
              driveLeftB.set(ControlMode.PercentOutput,-0.4);
              driveRightA.set(ControlMode.PercentOutput,0.4);
              driveRightB.set(ControlMode.PercentOutput,0.4);
          }        
          driveLeftA.set(ControlMode.PercentOutput,0);
          driveLeftB.set(ControlMode.PercentOutput,0);
          driveRightA.set(ControlMode.PercentOutput,0);
          driveRightB.set(ControlMode.PercentOutput,0);
          armUp = true;
          flag = true;
        }
        if (flag)
        {
          if(autoTimeElapsed < 10.5)
          {
            driveLeftA.set(ControlMode.PercentOutput,0.75);
            driveLeftB.set(ControlMode.PercentOutput,0.75);
            driveRightA.set(ControlMode.PercentOutput,0.75);
            driveRightB.set(ControlMode.PercentOutput,0.75);
          }
          else if(autoTimeElapsed < 12)
          {
            driveLeftA.set(ControlMode.PercentOutput,0.75);
            driveLeftB.set(ControlMode.PercentOutput,0.75);
            driveRightA.set(ControlMode.PercentOutput,0.75);
            driveRightB.set(ControlMode.PercentOutput,0.75);
            intake.set(ControlMode.PercentOutput, 1);
          }
          else
          {
            driveLeftA.set(ControlMode.PercentOutput,-0.75);
            driveLeftB.set(ControlMode.PercentOutput,-0.75);
            driveRightA.set(ControlMode.PercentOutput,-0.75);
            driveRightB.set(ControlMode.PercentOutput,-0.75);
          }
        }



        // else if(autoTimeElapsed < 30000)
        // {
        //   // intake.set(ControlMode.PercentOutput, 1);
        //   driveLeftA.set(ControlMode.PercentOutput,0.75);
        //   driveLeftB.set(ControlMode.PercentOutput,0.75);
        //   driveRightA.set(ControlMode.PercentOutput,0.75);
        //   driveRightB.set(ControlMode.PercentOutput,0.75);
        // }
        // else if(autoTimeElapsed < 6)
        // {
        //   intake.set(ControlMode.PercentOutput, 1);
        // }
        // else if(autoTimeElapsed < 8)
        // {
        //   driveLeftB.set(ControlMode.PercentOutput,-0.75);
        //   driveLeftA.set(ControlMode.PercentOutput,-0.75);
        //   driveRightA.set(ControlMode.PercentOutput,-0.75);
        //   driveRightB.set(ControlMode.PercentOutput,-0.75);
        // }
        // else if(autoTimeElapsed < 10 && ahrs.getAngle() > -5)
        // {
        //   //turnController.setSetpoint(179.9f);
        //   armUp = true;
        //   driveLeftA.set(ControlMode.PercentOutput,0.5);
        //   driveLeftB.set(ControlMode.PercentOutput,0.5);
        //   driveRightA.set(ControlMode.PercentOutput,-0.5);
        //   driveRightB.set(ControlMode.PercentOutput,-0.5);
        // }
        // else if(autoTimeElapsed < 10.5)
        // {
        //   driveLeftA.set(ControlMode.PercentOutput,-0.4);
        //   driveLeftB.set(ControlMode.PercentOutput,-0.4);
        //   driveRightA.set(ControlMode.PercentOutput,-0.4);
        //   driveRightB.set(ControlMode.PercentOutput,-0.4);
        // }
        // else if(autoTimeElapsed < 12.5)
        // {
        //   driveLeftA.set(ControlMode.PercentOutput,0);
        //   driveLeftB.set(ControlMode.PercentOutput,0);
        //   driveRightA.set(ControlMode.PercentOutput,0);
        //   driveRightB.set(ControlMode.PercentOutput,0);
        //   intake.set(ControlMode.PercentOutput, -1);
        // }
        break;
      case 3:
        // turnController.setSetpoint(100f);
        // currentRotationRate = MathUtil.clamp(turnController.calculate(ahrs.getAngle()), -1.0, 1.0);
        
        // intake.set(ControlMode.PercentOutput, -1);
        if(ahrs.getAngle() < 180-20)
        {
          driveLeftA.set(ControlMode.PercentOutput,-0.4);
          driveLeftB.set(ControlMode.PercentOutput,-0.4);
          driveRightA.set(ControlMode.PercentOutput,0.4);
          driveRightB.set(ControlMode.PercentOutput,0.4);
        }
        else
        {
          driveLeftA.set(ControlMode.PercentOutput,0);
          driveLeftB.set(ControlMode.PercentOutput,0);
          driveRightA.set(ControlMode.PercentOutput,0);
          driveRightB.set(ControlMode.PercentOutput,0);
        }
        break;



    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Set up arcade steer
    
    //Normal Arcade
    // double forward = driverController.getRawAxis(1);
    // forward *= Math.abs(forward);
    // double turn = driverController.getRawAxis(0);
    // turn *= Math.abs(turn);

    //Normalized input
    double throttleInput = driverController.getRawAxis(1) * 0.9;
    double turnInput = driverController.getRawAxis(4) * 0.6;
    double saturatedInput;
    double greaterInput = Math.max(Math.abs(throttleInput), Math.abs(turnInput));
      //range [0, 1]
    double lesserInput = Math.abs(throttleInput) + Math.abs(turnInput) - greaterInput;
      //range [0, 1]
    if (greaterInput > 0.0)
    {
      saturatedInput = (lesserInput / greaterInput) + 1.0;
        //range [1, 2]
    }
    else
    {
      saturatedInput = 1.0;
    }
    
    //scale down the joystick input values
    //such that (throttle + turn) always has a range [-1, 1]
    throttleInput = throttleInput / saturatedInput;
    turnInput = turnInput / saturatedInput;
		double driveLeftPower = throttleInput - turnInput;
		double driveRightPower = throttleInput + turnInput;


    driveLeftA.set(ControlMode.PercentOutput,driveLeftPower);
    driveLeftB.set(ControlMode.PercentOutput,driveLeftPower);
    driveRightA.set(ControlMode.PercentOutput,driveRightPower);
    driveRightB.set(ControlMode.PercentOutput,driveRightPower);

    // //Intake controls
    if(driverController.getRawButton(5)){
      intake.set(TalonSRXControlMode.PercentOutput, 1);;
    }
    else if(driverController.getRawButton(6)){
      intake.set(TalonSRXControlMode.PercentOutput, -1);
    }
    else{
      intake.set(TalonSRXControlMode.PercentOutput, 0);
    }

    //Arm Controls
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldDown);
      }
    }
  
    if(driverController.getRawButtonPressed(4) && !armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true;
    }
    else if(driverController.getRawButtonPressed(1) && armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = false;
    }  

  }

  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    driveLeftA.set(ControlMode.PercentOutput,0);
    driveLeftB.set(ControlMode.PercentOutput,0);
    driveRightA.set(ControlMode.PercentOutput,0);
    driveRightB.set(ControlMode.PercentOutput,0);
    // arm.set(0);
    // intake.set(ControlMode.PercentOutput, 0);
  }
    
}