// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //private final Subsystems my_subsystem = new Subsystems();
  private final WPI_TalonFX shooter_motor1 = new WPI_TalonFX(7);
  private final WPI_TalonFX shooter_motor2 = new WPI_TalonFX(8);

  private final WPI_TalonFX driver_leftmotor1 = new WPI_TalonFX(2);
  private final WPI_TalonFX driver_leftmotor2 = new WPI_TalonFX(3);
  private final WPI_TalonFX driver_rightmotor1 = new WPI_TalonFX(4);
  private final WPI_TalonFX driver_rightmotor2 = new WPI_TalonFX(5);
  private final WPI_TalonFX climber_motor1 = new WPI_TalonFX(14);
  private final WPI_TalonFX climber_motor2 = new WPI_TalonFX(15);

  private final WPI_TalonFX intake_motor1 = new WPI_TalonFX(19);
  private final WPI_TalonFX conveyer1 = new WPI_TalonFX(20);
  
  //Joysticks
  private final Joystick driver_joystick = new Joystick(0);
  //private final Joystick copilot_joystick = new Joystick(1);
  DifferentialDrive tarzan_robot = new DifferentialDrive(driver_leftmotor1, driver_rightmotor1);

  // Creates UsbCamera
  UsbCamera driver_camera = new UsbCamera("USB Camera 0", 0);

  /// Setup the digital inputs
  private final DigitalInput conveyor_loc_1 = new DigitalInput(0);

  // Setup the color sensor
  private final ColorSensorV3 color_sensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

  private Robot_Cargo_State cargo_status = Robot_Cargo_State.Idle;
  private final Timer state4_Timer = new Timer();
  private double tx_angle;
  private double ty_angle = -1000.0;

  @Override
  public void robotInit() {

    shooter_motor1.configFactoryDefault();
    shooter_motor2.configFactoryDefault();

    driver_leftmotor1.configFactoryDefault();
    driver_leftmotor2.configFactoryDefault();
    driver_rightmotor1.configFactoryDefault();
    driver_rightmotor2.configFactoryDefault();

    climber_motor1.configFactoryDefault();
    climber_motor2.configFactoryDefault();

    intake_motor1.configFactoryDefault();
    conveyer1.configFactoryDefault();

    //Mechanism Inversion settings 
    shooter_motor2.setInverted(true);
    driver_rightmotor1.setInverted(true);
    driver_rightmotor2.setInverted(true);
    climber_motor2.setInverted(true);
    conveyer1.setInverted(true);
    intake_motor1.setInverted(true);

    //Shooter and Driver Follow
    shooter_motor2.follow(shooter_motor1);
    driver_rightmotor2.follow(driver_rightmotor1);
    driver_leftmotor2.follow(driver_leftmotor1);

    //Climber Follow
    climber_motor2.follow(climber_motor1);

    // Shooter Control Loop Settings
    shooter_motor1.configNeutralDeadband(0.001);
		shooter_motor1.config_kP(0, 0.015, 30);
		shooter_motor1.config_kI(0, 0.000, 30);
		shooter_motor1.config_kD(0, 0, 30);
    shooter_motor1.config_kF(0, 2048/22000, 30);

    //Front camera one time setup
    CameraServer.startAutomaticCapture();

    //Setup color sensor
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);

  }

  @Override
  public void robotPeriodic() {}
    
  @Override
  public void autonomousInit() {
    state4_Timer.start();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
    cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
  }

  @Override
  public void autonomousPeriodic() {
    tx_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    if (camAngletoDistance(ty_angle) <= 10){
      tarzan_robot.tankDrive(-1*0.8, -1*0.8);
    }
    else if (camAngletoDistance(ty_angle) > 10){
      if (Math.abs(tx_angle) > 0.1){
        tarzan_robot.tankDrive(-1*tx_angle, 1*tx_angle);
      }
      else{
        tarzan_robot.tankDrive(0, 0);
      }
    }
    if (cargo_status == Robot_Cargo_State.Cargo_awaiting_shooter){
      shooter_motor1.set(ControlMode.Velocity, 18000);
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)){
        cargo_status = Robot_Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_being_shot)
    {
      state4_Timer.start();
      conveyer1.set(0.8); //running conveyer 
      if (state4_Timer.get() > 2){
        conveyer1.set(0);
        shooter_motor1.set(0);
        state4_Timer.stop();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }
  }

  @Override
  public void teleopInit() {
    state4_Timer.start();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
  }

  @Override
  public void teleopPeriodic() {
        //Tank Drive
        tarzan_robot.tankDrive(-1*driver_joystick.getRawAxis(1), -1*driver_joystick.getRawAxis(5));

        //Climber
        //climber_motor1.set(0.5*driver_joystick.getRawAxis(5));

        //Intake (positive inputs intake a cargo)

        if(driver_joystick.getRawButton(5) == true){
          intake_motor1.set(-1);
        }
        else if(driver_joystick.getRawButton(5) == false){
          intake_motor1.set(driver_joystick.getRawAxis(2));
        }
        

        //Conveyor (positive inputs bring cargo in)
        if ((driver_joystick.getRawButton(4) == true) && (driver_joystick.getRawButton(2) == false)) {
          conveyer1.set(1);
        }
        else if((driver_joystick.getRawButton(4) == false) && driver_joystick.getRawButton(2) == true){
          conveyer1.set(-1);
        }
        else {
          conveyer1.set(0);
        }

        //Shooter (positive inputs shoot cargo out)
        shooter_motor1.set(driver_joystick.getRawAxis(3)*0.8);
        
        // Read color sensor
        Color detectedColor = color_sensor.getColor();
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kBlueTarget) {
          colorString = "Blue";
        } else if (match.color == kRedTarget) {
          colorString = "Red";
        }
        else {
          colorString = "Unknown";
        }
        SmartDashboard.putString("color sensor output", colorString);
        SmartDashboard.putNumber("Timer", state4_Timer.get());

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  // This subroutine performs a semi-autonomous intake and shooting process
  public void autoIntake() {
    if((cargo_status == Robot_Cargo_State.Idle) && (driver_joystick.getRawButton(6) == true)){
      cargo_status = Robot_Cargo_State.Cargo_being_intaked;
    }

    if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == true)) {
        intake_motor1.set(0.8); //running intake
        conveyer1.set(0.8); //running conveyer
        //shooter_motor1.set(1*0.8); //starting shooter at 80%
        shooter_motor1.set(ControlMode.Velocity, 18000);
    } 
    else if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == false)) {
      cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
      intake_motor1.set(0); //stopping intake
      conveyer1.set(0); //stopping conveyer
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_awaiting_shooter){
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)){
        cargo_status = Robot_Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_being_shot)
    {
      state4_Timer.start();
      conveyer1.set(0.8); //running conveyer 
      if (state4_Timer.get() > 2){
        conveyer1.set(0);
        shooter_motor1.set(0);
        state4_Timer.stop();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }  
  }

  // This is is a custom type used to track the state of Cargo intake and shooting
  public enum Robot_Cargo_State {
    Idle,    // This state means that the robot has no cargo in it and all intake/conveyor/shooter motors are off
    Cargo_being_intaked,     // This state means that a cargo is in the process of being intaked, but still in transit
    Cargo_awaiting_shooter,    // This state means that a cargo is in the robot and awaiting to be shot out
    Cargo_being_shot,    // This state means that the cargo is being shot out
    Error   // This is an error state or condition
  }

  // This method converts a target pitch angle into an estimated robot distance away from the target
  double camAngletoDistance(double camAngle) {
    return 0.0;
  }

}