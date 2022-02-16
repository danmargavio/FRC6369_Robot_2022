// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Joystick;

//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;

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

  //private final UsbCamera frontCamera = new UsbCamera("front_camera", 1);
  
  //Joysticks
  private final Joystick driver_joystick = new Joystick(0);
  //private final Joystick copilot_joystick = new Joystick(1);
  DifferentialDrive tarzan_robot = new DifferentialDrive(driver_leftmotor1, driver_rightmotor1);
  
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

    //Front camera one time setup
    //CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {}
    
  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

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
        
        
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

}