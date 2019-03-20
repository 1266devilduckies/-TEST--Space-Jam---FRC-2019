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
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private double m_maxSpeed = 1;

  private final Talon m_extraMotor = new Talon (7);
  private double m_maxExtraSpeed = 1;

  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_stick2 = new Joystick(1);
  
  private final Timer m_timer = new Timer();

  private boolean visionToggle = false;
  SerialPort usbSerial = null;

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private int turnCount = 0;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kF = 0;
  private double integral, previous_error, stickAngle = 0;

  private double timer_start = 0;
  private boolean climbing = false;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //CameraServer.getInstance().startAutomaticCapture();
    //m_stick.setXChannel(2);
    SmartDashboard.putNumber("Maximum Drive Speed", m_maxSpeed);
    SmartDashboard.putNumber("Maximum Motor Speed", m_maxExtraSpeed);
    SmartDashboard.putNumber("drive_kP", kP);
    SmartDashboard.putNumber("drive_kI", kI);
    SmartDashboard.putNumber("drive_kD", kD);
    SmartDashboard.putNumber("drive_kF", kF);
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
    double error = 0;

    kP = SmartDashboard.getNumber("kP", 0);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0);
    kF = SmartDashboard.getNumber("kF", 0);

    if(m_stick.getRawButtonPressed(13)){
      m_gyro.reset();
    }

    if(m_stick.getRawButtonPressed(6)==true){
      visionToggle = !visionToggle;
    }

    if(usbSerial != null && visionToggle == true){
      if(usbSerial.getBytesReceived()>0){
        JsonArray jevoisArray = Jsoner.deserialize(usbSerial.readString(), new JsonArray());
        if(jevoisArray.isEmpty()==false){
          error = jevoisArray.getDouble(0);
          //gyro = m_gyro.getAngle();
        }
      }
    }else{
      gyro = m_gyro.getAngle() - (360*turnCount);

      if(gyro>180){
        turnCount++;
      }else if (gyro<=-180){
        turnCount--;
      }

      gyro = m_gyro.getAngle() - (360*turnCount);
      
      if (Math.abs( m_stick.getX() ) > 0.1 || Math.abs( m_stick.getY() ) > 0.1){
        stickAngle = m_stick.getDirectionDegrees();
      }

      error = stickAngle - gyro;
    }

      integral += (error*0.02);
      double derivative = (error-previous_error)/0.02; 
      double turn_power = (kP * error) + (kI*integral) + (kD*derivative) + kF;

      if(turn_power > m_maxSpeed){
        turn_power = m_maxSpeed;
      }else if (turn_power < m_maxSpeed*-1){
        turn_power = m_maxSpeed*-1;
      }
      
      previous_error = error;

      //m_robotDrive.arcadeDrive(m_stick.getRawAxis(1)*m_maxSpeed*-1,turn_power,false);
      m_robotDrive.arcadeDrive(m_stick.getRawAxis(1)*m_maxSpeed*-1, m_stick.getRawAxis(2)*m_maxSpeed);
      //m_robotDrive.curvatureDrive(m_stick.getRawAxis(1)*m_maxSpeed*-1, m_stick.getRawAxis(2), m_stick.getRawButton(6));

      SmartDashboard.putNumber("Gyro Angle", gyro);
      SmartDashboard.putBoolean("Gyro Connected", m_gyro.isConnected());
      SmartDashboard.putNumber("Error", error);
      SmartDashboard.putNumber("Turn Power", turn_power);
    
    if(SmartDashboard.getNumber("Maximum Drive Speed", 1)<=1 && SmartDashboard.getNumber("motorMaxSpeed", 1)>=0){
      m_maxSpeed = SmartDashboard.getNumber("Maximum Drive Speed", 1);
    }

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
    System.out.println(m_timer.getFPGATimestamp());
    
    if(m_stick.getRawButtonPressed(13)){
      climbing = !climbing;
      timer_start = m_timer.getFPGATimestamp();
    }

    if(climbing){
      double current_time = m_timer.getFPGATimestamp() - timer_start;

      if(current_time <= 2){
        SmartDashboard.putString("climbStat", "lowering elevator and lowering climber");
      }else if(current_time <= 4){
        SmartDashboard.putString("climbStat", "move forward with wheels and claw");
      }else if(current_time <= 6){
        SmartDashboard.putString("climbStat", "lift climber back up");
      }
    }
  }
}
