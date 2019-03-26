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

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.*;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

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

  private double kP = 1.5;
  private double kI = 0;
  private double kD = 0;
  private double kF = 0.03;
  private double integral, previous_error, stickAngle = 0;

  private final Talon m_extraMotor = new Talon (7);
  private double m_maxExtraSpeed = 1;

  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_stick2 = new Joystick(1);
  private double stickyY = 0;

  private final Timer m_timer = new Timer();

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private int turnCount = 0;

  SerialPort usbSerial = null;
  private boolean visionToggle = true;
  private boolean seesTarget = false;
  private double targetBlind_count = 0;
  private double targetBlind_threshold = 0.3;
  private int targetSelect = 0;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //CameraServer.getInstance().startAutomaticCapture();
    m_stick.setXChannel(0);
    m_stick.setYChannel(1);
    SmartDashboard.putNumber("Maximum Drive Speed", 1);
    SmartDashboard.putNumber("Maximum Motor Speed", 1);
    SmartDashboard.putNumber("drive_kP", kP);
    SmartDashboard.putNumber("drive_kI", kI);
    SmartDashboard.putNumber("drive_kD", kD);
    SmartDashboard.putNumber("drive_kF", kF);
    m_gyro.reset();
    m_gyro.calibrate();
    
    try {
			System.out.print("Creating JeVois SerialPort...");
			usbSerial = new SerialPort(115200,SerialPort.Port.kUSB1);
			System.out.println("SUCCESS!!");
		} catch (Exception e) {
			System.out.println("FAILED!!  Fix and then restart code...");
                        e.printStackTrace();
    }

    /*
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 240, 15);
      
      Mat sourceImg = new Mat();
      Mat output = new Mat();

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 320, 240);
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(sourceImg);
          Imgproc.cvtColor(sourceImg, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
    }).start();
*/
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
    double turnPower = 0;
    String selectionOutput = "[[LEFT]]";
    JsonArray jevoisArray = null;
    double multiplePair_count = 0;
    double multiplePair_threshold = 0.3;

    kP = SmartDashboard.getNumber("drive_kP", 0);
    kI = SmartDashboard.getNumber("drive_kI", 0);
    kD = SmartDashboard.getNumber("drive_kD", 0);
    kF = SmartDashboard.getNumber("drive_kF", 0);

    if(m_stick.getRawButtonPressed(13)){
      m_gyro.reset();
    }

    /*if(m_stick.getRawButtonPressed(6)==true){
      visionToggle = !visionToggle;
    }*/

    visionToggle = m_stick.getRawButton(6);

    //SmartDashboard.putString("stri", usbSerial.readString());

    double centerY = 0;
    double dist = -10000;

    if(usbSerial != null){
      if(usbSerial.getBytesReceived() > 0){
       
        jevoisArray = Jsoner.deserialize(usbSerial.readString(), new JsonArray());

        if(!jevoisArray.isEmpty()){

          targetBlind_count = 0;

          if(jevoisArray.size()==6){
            multiplePair_count += 0.02;
          }else{
            multiplePair_count = 0;
          }

          if(multiplePair_count > multiplePair_threshold){

            if(m_stick.getRawButtonPressed(5)){
              if(targetSelect == 0){
                targetSelect = 3;
              }else{
                targetSelect = 0;
              }
            }

            if(targetSelect==0){
              selectionOutput = "[[ LEFT ]]       RIGHT   ";
            }else if(targetSelect==3){
              selectionOutput = "   LEFT       [[ RIGHT ]]";
            }

          }else{
            targetSelect = 0;
              selectionOutput = "        [[ SET ]]        ";
          }

          SmartDashboard.putNumber("centerY", jevoisArray.getDouble(targetSelect+1));

        }else{
          targetBlind_count += 0.02;
        }
      }
    }

    seesTarget = targetBlind_count < targetBlind_threshold ? true : false;

    targetSelect = seesTarget ? targetSelect : 0;

    if(visionToggle == true && jevoisArray != null){
      if(jevoisArray.isEmpty()==false){

        //order in array goes centerX, centerY, distance between both
        double centerX = jevoisArray.getDouble(targetSelect);
        centerY = jevoisArray.getDouble(targetSelect+1);
        double between = jevoisArray.getDouble(targetSelect+2);
        double width = (centerX*11.5)/between;
        if(/*m_photoElec.get() ==*/ false){
          dist = (0.0001*Math.pow(centerY, 2))-(0.087*centerY)+34.29;
        }else if(true){
          //dist = (0.008*Math.pow(centerY, 2))+(0.1828*centerY)+10.225;
          //targetIN*DegInPix / 2*(targetPixel)*tan(half of cameraFOV)
          dist = (11.5*0.2031)/(2*(centerX*160+160)*(Math.tan(Math.toRadians(32.5))));
        }else{
          double radians = Math.toRadians(between*0.2031);
          dist = ((11.5)/Math.tan(radians));
        }
        error = Math.atan(width/dist);
        //gyro = m_gyro.getAngle();
      }
    }else{
      gyro = m_gyro.getAngle() - (360*turnCount);

      if(gyro>180){
        turnCount++;
      }else if (gyro<=-180){
        turnCount--;
      }

      gyro = m_gyro.getAngle() - (360*turnCount);
      
      if (Math.abs( m_stick.getX() ) > 0.5 || Math.abs( m_stick.getY() ) > 0.5){
        stickAngle = Math.atan((m_stick.getY()*-1)/m_stick.getX());
      }

      error = stickAngle - gyro;

      SmartDashboard.putNumber("Stick Error", m_stick.getDirectionDegrees());
    }

    if(error > 180) {
      error-=360;
    }else if (error<-180){
      error+=360;
    }
    
    integral += (error*.02);
    double derivative = (error-previous_error)/.02;

    if(error != 0){
      turnPower = (kP * error) + (kI*integral) + (kD*derivative) + kF;
    }

    turnPower += turnPower > 0 ? kF : -kF; 

    previous_error = error;

    turnPower = turnPower > m_maxSpeed ? m_maxSpeed : turnPower < m_maxSpeed*-1 ? m_maxSpeed*-1 : turnPower;

    //m_robotDrive.arcadeDrive(m_stick.getRawAxis(1)*m_maxSpeed*-1, m_stick2.getRawAxis(0)*m_maxSpeed);
    //m_robotDrive.arcadeDrive(m_stick.getRawAxis(1)*m_maxSpeed*-1, m_stick.getRawAxis(2)*m_maxSpeed);
    //stickyY = Math.abs(m_stick2.getRawAxis(1)) > 0.4 ? m_stick2.getRawAxis(1) : 0;
    //m_robotDrive.arcadeDrive(stickyY,turnPower,false);

    if(visionToggle && seesTarget){
      m_robotDrive.arcadeDrive(m_stick.getRawAxis(1)*m_maxSpeed*-1, turnPower);
    }else{
      m_robotDrive.curvatureDrive(m_stick.getRawAxis(1)*m_maxSpeed*-1, m_stick.getRawAxis(2)*m_maxSpeed, m_stick.getRawButton(8));
    }

    SmartDashboard.putNumber("Gyro Angle", gyro);
    SmartDashboard.putBoolean("Gyro Connected", m_gyro.isConnected());
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("dist", dist);
    SmartDashboard.putNumber("Turn Power", turnPower);
    SmartDashboard.putBoolean("Vision Toggled On?", visionToggle);
    SmartDashboard.putBoolean("Sees Target?",seesTarget);
    SmartDashboard.putBoolean("Jevois Connected?", usbSerial != null);
  
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
  }
}
