/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final Talon m_leftMotor = new Talon (8);
  private final Talon m_rightMotor = new Talon (9);
  private final Talon m_extraMotor = new Talon (7);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_stick2 = new Joystick(1);
  private final DigitalInput m_ballSensor = new DigitalInput(0);
  private final Timer m_timer = new Timer();
  private double m_maxSpeed = 1;
  private double m_maxExtraSpeed = 1;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("Maximum Drive Speed", 1);
    SmartDashboard.putNumber("Maximum Motor Speed", 1);
    m_stick.setXChannel(2);
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    if(SmartDashboard.getNumber("Maximum Drive Speed", 1)<=1 && SmartDashboard.getNumber("motorMaxSpeed", 1)>=0){
      m_maxSpeed = SmartDashboard.getNumber("Maximum Drive Speed", 1);
    }

    m_robotDrive.arcadeDrive(m_stick.getY()*m_maxSpeed*-1, m_stick.getX()*m_maxSpeed);

    if(SmartDashboard.getNumber("Maximum Motor Speed", 1)<=1 && SmartDashboard.getNumber("motorMaxSpeed", 1)>=-1){
      m_maxExtraSpeed = SmartDashboard.getNumber("Maximum Motor Speed", 1);
    }

    double targetOutput = 0;

    if(m_stick.getRawButton(4)){
      targetOutput = m_maxExtraSpeed;
    }else if(m_stick.getRawButton(2)){
      targetOutput = m_maxExtraSpeed*-1;
    }else{
      targetOutput = 0;
    }

    if (m_extraMotor.getSpeed()>targetOutput){
      m_extraMotor.setSpeed(m_extraMotor.getSpeed() - 0.1);
    }else if (m_extraMotor.getSpeed()<targetOutput){
      m_extraMotor.setSpeed(m_extraMotor.getSpeed() + 0.1);
    }

    SmartDashboard.putNumber("Extra Motor Speed", m_extraMotor.getSpeed());
    SmartDashboard.putNumber("Right Motor Speed", m_rightMotor.getSpeed());
    SmartDashboard.putNumber("Left Motor Speed", m_leftMotor.getSpeed());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
      SmartDashboard.putBoolean("photoelectric", m_ballSensor.get());
  }
}
