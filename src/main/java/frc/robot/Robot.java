// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.filter.SlewRateLimiter;
/* default imports */
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
 // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/* controller imports */
/* spark max improts */
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/* victor imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
*/

//import SmartDashboard
//import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  /* setup motors */
  // drive motors
  CANSparkMax m_DriveLeftSpark1 = new CANSparkMax(1, MotorType.kBrushed);
 // VictorSPX m_DriveLeftVictor = new VictorSPX(1);
  CANSparkMax m_DriveLeftSpark2 = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax m_DriveRightSpark3 = new CANSparkMax(3, MotorType.kBrushed);
 // VictorSPX m_DriveRightVictor = new VictorSPX(3);
  CANSparkMax m_DriveRightSpark4 = new CANSparkMax(4, MotorType.kBrushed);
  // intake motors
  CANSparkMax m_IntakeSpark = new CANSparkMax(6, MotorType.kBrushless);
  // arm motors
  CANSparkMax m_ArmSpark = new CANSparkMax(5, MotorType.kBrushless);

  XboxController driveController = new XboxController(0);
  XboxController opController = new XboxController(1);


  static final int ARM_CURRENT_LIMIT_A = 20;
  static final double ARM_OUTPUT_POWER = 0.4;

  /* globalish variables */
  double autoTimeElapsed = 0;

  //SlewRateLimiter forwardFilter;
  //SlewRateLimiter turnFilter;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //SmartDashboard smartDashboard = new SmartDashboard();

    // SlewRateLimiter forwardFilter will limit forward acceleration to 0.5 units per loop
  //  forwardFilter = new SlewRateLimiter(0.25, -0.25, 0);
  //  turnFilter = new SlewRateLimiter(0.5, -0.5, 0);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    /* Configure motors */
    // configure drive motors
    m_DriveLeftSpark1.setInverted(false);
    m_DriveLeftSpark2.setInverted(false);
    m_DriveRightSpark3.setInverted(true);
    m_DriveRightSpark4.setInverted(true);
    // configure intake motors
    m_IntakeSpark.setInverted(false);
    // configure arm motors
    m_ArmSpark.setInverted(false);
    //m_ArmSpark.setIdleMode(IdleMode.kBrake);

    

  }

  @Override
  public void robotPeriodic() {
  }

  double autoStartTime;

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    autoStartTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autoTimeElapsed = Timer.getFPGATimestamp() - autoStartTime;

    if (autoTimeElapsed < 3) {
      m_DriveLeftSpark1.set(0.3);
      m_DriveLeftSpark2.set(0.3);
      m_DriveRightSpark3.set(0.3);
      m_DriveRightSpark4.set(0.3);

 //   } else if (autoTimeElapsed < 4) {
    } else if (autoTimeElapsed < 4) {
      m_DriveLeftSpark1.set(-0.3);
      m_DriveLeftSpark2.set(-0.3);
      m_DriveRightSpark3.set(-0.3);
      m_DriveRightSpark4.set(-0.3);
      
      m_DriveLeftSpark1.set(0.0);
      m_DriveLeftSpark2.set(0.0);
      m_DriveRightSpark3.set(0.0);
      m_DriveRightSpark4.set(0.0);
    }
  }  

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  // This function is called periodically during teleoperated mode.//
  @Override
  public void teleopPeriodic() {
    /* drive Controls */

    double forward = driveController.getLeftY();
    double turn = -driveController.getRightX();

    double leftSpeed = forward + turn;
    double rightSpeed = forward - turn;

    double intakeSpeed = opController.getRightY();

    double armSpeed = opController.getLeftY();
/*
    double forward;
    double turn;

    if (driveController.getLeftY() >= 0.00 ||
        driveController.getLeftY() < -0.0) {
      forward = driveController.getLeftY();
    }
    else{
      forward = 0.0;
    }

    if (driveController.getRightX() >= 0.00 ||
        driveController.getRightX() < -0.00) {
      turn = -driveController.getRightX();
    }
    else{
      turn = 0.0;
    }*/


    //double TestFilteredForward = forwardFilter.calculate(forward); // + turn;
    //String StringTestFilteredForward = Double.toString(TestFilteredForward);
    //double TestFilteredRightSpeed = forwardFilter.calculate(forward); // - turn; 
    //boolean test = SmartDashboard.putString("Test Filterded Forward Speed is: ", StringTestFilteredForward);

   //System.out.println("Raw Speed is: "+forward+" and Filterded Right Speed is: "+ TestFilteredForward);

    //double leftSpeed = forward + turn;
    //double rightSpeed = forward - turn;

    // Print speed to console (not sure if this will work)
    /*System.out.print("Left Speed is: " + leftSpeed);
    System.out.print("Right Speed is: "+ rightSpeed);

    // Raw values from controller
    System.out.print("getLeftY value is " + driveController.getLeftY());
    System.out.print("getRightX value is "+ driveController.getRightX());
    System.out.print("getLeftX value is " + driveController.getLeftX());
    System.out.print("getRightY value is "+ driveController.getRightY());*/

    m_DriveLeftSpark1.set(leftSpeed);
    m_DriveLeftSpark2.set(leftSpeed);
    m_DriveRightSpark3.set(rightSpeed);
    m_DriveRightSpark4.set(rightSpeed);

    // Set Intake
    m_IntakeSpark.set(intakeSpeed);
    

    // Set Arm
    m_ArmSpark.set(armSpeed);

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
