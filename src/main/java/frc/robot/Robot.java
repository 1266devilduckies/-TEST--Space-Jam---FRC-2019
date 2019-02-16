/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.github.cliftonlabs.json_simple.*;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import java.math.BigDecimal;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.cscore.*;

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
  private final Timer m_timer = new Timer();
  private double m_maxSpeed = 1;
  private double m_maxExtraSpeed = 1;

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private int turnCount = 0;
  private double kP = 0;
  private double kI = 0;

  SerialPort usbSerial = null;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //CameraServer.getInstance().startAutomaticCapture();
    m_stick.setXChannel(2);
    SmartDashboard.putNumber("Maximum Drive Speed", 1);
    SmartDashboard.putNumber("Maximum Motor Speed", 1);
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kI", 0);
    m_gyro.reset();
    m_gyro.calibrate();
    
    try {
			System.out.print("Creating JeVois SerialPort...");
			usbSerial = new SerialPort(115200,SerialPort.Port.kUSB);
			System.out.println("SUCCESS!!");
		} catch (Exception e) {
			System.out.println("FAILED!!  Fix and then restart code...");
                        e.printStackTrace();
		}
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

  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    double gyro = 0;
    double error = -1000;
    double integral = 0;
    double turn_power = 0;
    kP = SmartDashboard.getNumber("kP", 0);
    kI = SmartDashboard.getNumber("kI", 0);

    if(m_stick.getRawButtonPressed(13)){
      m_gyro.reset();
    }

    if(usbSerial != null){

      if(usbSerial.getBytesReceived()>0){
        JsonArray jevoisArray = Jsoner.deserialize(usbSerial.readString(), new JsonArray());
        if(jevoisArray.isEmpty()==false){
          double angleError = jevoisArray.getDouble(0);
          gyro = m_gyro.getAngle() - (360*turnCount);

          if(gyro>180){
            turnCount++;
          }else if (gyro<=-180){
            turnCount--;
          }

          gyro = m_gyro.getAngle() - (360*turnCount);

          if (Math.abs( m_stick.getX() ) > 0.1 || Math.abs( m_stick.getY() ) > 0.1){

            error = m_stick.getDirectionDegrees() - (gyro);

            if(error > 180) {
              error-=360;
            }else if (error<-180){
              error+=360;
            }
            integral += (error*.02);
          }

          if(error > -1000){
            turn_power = (kP * error) + (kI*integral);
          }

          if(turn_power > m_maxSpeed){
            turn_power = m_maxSpeed;
          }else if (turn_power < m_maxSpeed*-1){
            turn_power = m_maxSpeed*-1;
          }

          SmartDashboard.putNumber("Stick Angle", m_stick.getDirectionDegrees());
          SmartDashboard.putNumber("Gyro Angle", gyro);
          SmartDashboard.putBoolean("Gyro Connected", m_gyro.isConnected());
          SmartDashboard.putNumber("Error", error);
          SmartDashboard.putNumber("Turn Power", turn_power);

          //m_robotDrive.arcadeDrive(0, 0);
          m_robotDrive.arcadeDrive(m_stick.getZ()*m_maxSpeed*-1,turn_power,false);
        }
      }

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
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
