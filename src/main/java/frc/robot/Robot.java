// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import java.lang.management.CompilationMXBean;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
//import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

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

  private final DigitalInput climberEncoderData = new DigitalInput(1);
  private final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(climberEncoderData);
  //Joysticks
  private final Joystick driver_joystick = new Joystick(0);
  private final Joystick copilot_joystick = new Joystick(1);
  DifferentialDrive tarzan_robot = new DifferentialDrive(driver_leftmotor1, driver_rightmotor1);

  // Creates UsbCamera
  UsbCamera driver_camera = new UsbCamera("USB Camera 0", 0);

  /// Setup the digital inputs
  private final DigitalInput conveyor_loc_1 = new DigitalInput(0);

  // Setup the pneumatics devices, 
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid IntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15); /* Make sure channel number associates with kReverse and Forward Ex: Channel 6 brings down (kReverse) and vice versa with channel 7*/
  DoubleSolenoid LeftClimberSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 11); //Kforward is Retract and KReverse is Extend
  DoubleSolenoid RightClimberSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 14);
  DoubleSolenoid LeftClimberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 13);
  DoubleSolenoid RightClimberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 12);
  

  // Setup the color sensor
  //private final ColorSensorV3 color_sensor = new ColorSensorV3(I2C.Port.kOnboard);
  //private final ColorMatch m_colorMatcher = new ColorMatch();

  //private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  //private final Color kRedTarget = new Color(0.561, 0.100, 0.340);   // Dan adjusted these values based on measurements of the cargo

  private Robot_Cargo_State cargo_status = Robot_Cargo_State.Idle;
  private Intake_Deployment_State intake_status = Intake_Deployment_State.up;
  private Climber_State Climber_status = Climber_State.start;
  private final Timer state4_Timer = new Timer();
  private final Timer state2_Timer = new Timer();
  private double tx_angle;
  private double ty_angle = -1000.0; //target degrees above the center of the camera 

  private final double cameraPitch = 18.104; //degrees above horizon ||
  private final double pupilCameraHeight = 32.5; //inches above the ground ||
  private final double goalHeight = 104; //inches above the ground to the top of the goal
  private double distanceFromGoal = 0; //inches parallel from shooter to the center of the goal
  private final double goalRadius = 26.7716535; //inches 
  private final double pupilDistanceToShooter = -6; //inches, in relation to distance from goal ||
  private final double desiredDistanceFromGoal = 168; //inches, distance from the shooter to the center of goal (114.75in - 24in) ||
  private double pressureValue = 0;
  
  
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


    climber_motor1.configNeutralDeadband(0.001);
		climber_motor1.config_kP(0, 0.015, 30);
		climber_motor1.config_kI(0, 0.000, 30);
		climber_motor1.config_kD(0, 0, 30);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);

    //Front camera one time setup
    //CameraServer.startAutomaticCapture();

    //Setup color sensor
    //m_colorMatcher.addColorMatch(kBlueTarget);
    //m_colorMatcher.addColorMatch(kRedTarget);

    //Setup compressor controls for analog pressure transducer
    phCompressor.enableAnalog(90, 120);
    phCompressor.enabled();

    //Initializes Solenoids on position 'A'
    IntakeSolenoid.set(Value.kForward);
    RightClimberSolenoid1.set(Value.kForward);
    RightClimberSolenoid2.set(Value.kForward);
    LeftClimberSolenoid1.set(Value.kForward);
    LeftClimberSolenoid2.set(Value.kForward);

    //climberEncoder.setPositionOffset();
  }

  @Override
  public void robotPeriodic() {

    //SmartDashboard.putNumber("RED", color_sensor.getRed());
    //SmartDashboard.putNumber("BLUE", color_sensor.getBlue());
    //SmartDashboard.putNumber("GREEN", color_sensor.getGreen());
    pressureValue = phCompressor.getPressure();
    SmartDashboard.putNumber("PSI", pressureValue);
    SmartDashboard.putBoolean("Ball In", conveyor_loc_1.get());
    SmartDashboard.putNumber("Climber Arm Position", climberEncoder.get());
    SmartDashboard.putNumber("distance", camAngletoDistance(ty_angle));
    SmartDashboard.putNumber("Velocity of shooter", shooter_motor1.getSelectedSensorVelocity());
    tx_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    
    
  }
    
  @Override
  public void autonomousInit() {
    state4_Timer.start();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
    cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
  }

  @Override
  public void autonomousPeriodic() {
    moveIntakeUptoDown();

    if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (camAngletoDistance(ty_angle) <= desiredDistanceFromGoal)) {
      tarzan_robot.tankDrive(0.4, 0.4);
    }
    else if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (camAngletoDistance(ty_angle) > desiredDistanceFromGoal)) {
      /*if (Math.abs(tx_angle) > 0.5){
        tarzan_robot.tankDrive(-1*tx_angle, 1*tx_angle);
      }*/
      //autoAim();
      cargo_status = Robot_Cargo_State.Idle;
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_awaiting_shooter) {
      shooter_motor1.set(0.8);
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)) {
        state4_Timer.start();
        cargo_status = Robot_Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_being_shot) {
      conveyer1.set(0.8); //running conveyer 
      if (state4_Timer.get() > 2.0){
        conveyer1.set(0);
        shooter_motor1.set(0);
        state4_Timer.stop();
        cargo_status = Robot_Cargo_State.Cargo_being_intaked;
      }
    }
  }

  @Override
  public void teleopInit() {
    state4_Timer.start();
  }

  @Override
  public void teleopPeriodic() {
        //climber test
        //climberTest();
        //Compressor Test
        //compressorTest();
        if (copilot_joystick.getRawButton(7) && copilot_joystick.getRawButton(3)){
          moveIntakeDowntoUp();
        }
        if (copilot_joystick.getRawButton(7) && copilot_joystick.getRawButton(1)){
          moveIntakeUptoDown();
        }

        //If Driver is controlling, don't auto aim, but if driver presses button they are forced to switch to auto aiming
        if (driver_joystick.getRawButton(2)){
          autoAim();

        }
        else{
          tarzan_robot.tankDrive(-1*driver_joystick.getRawAxis(1), -1*driver_joystick.getRawAxis(5));
        }
        //Intake (positive inputs intake a cargo)
        if (intake_status == Intake_Deployment_State.down){
          //autoIntake(); // currently replaces manualIntake();
          manualIntake(); 
          
          //IntakeTest1();
          //IntakeReverseTest();
        }

        // Read color sensor
        /*Color detectedColor = color_sensor.getColor();
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
        SmartDashboard.putString("color sensor output", colorString); */
        SmartDashboard.putNumber("Timer", state2_Timer.get());

        if (driver_joystick.getRawButton(7) && driver_joystick.getRawButton(8)){
          moveIntakeDowntoUp();
        }
        if (driver_joystick.getRawButton(7) && driver_joystick.getRawButton(3)){
          initiateMiddleRungClimb();
        }
        if (driver_joystick.getRawButton(7) && driver_joystick.getRawButton(1)){
          
          finalizeMiddleRungClimb();
        }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    cargo_status = Robot_Cargo_State.Idle;
  }

  @Override
  public void testPeriodic() {
    compressorTest();
    climberTest();
    manualIntake();
  }

    /**
   * This subroutine performs the semi-autonomous intake and shooting process
   *
   */
  public void autoIntake() {
    if((cargo_status == Robot_Cargo_State.Idle) && (driver_joystick.getRawButton(6) == true)){
      cargo_status = Robot_Cargo_State.Cargo_being_intaked;
      state2_Timer.start();
    }

    if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == true)) {
      if (driver_joystick.getRawButton(6) == true){
        state2_Timer.reset();
      }
      intake_motor1.set(0.8); //running intake
      conveyer1.set(0.8); //running conveyer
      //shooter_motor1.set(1*0.8); //starting shooter at 80%
      shooter_motor1.set(ControlMode.Velocity, 18000);
      if (state2_Timer.get() > 4.0) {
        intake_motor1.set(0);
        conveyer1.set(0);
        shooter_motor1.set(0);
        state2_Timer.stop();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }
    else if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == false)) {
      cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
      state2_Timer.stop();
      intake_motor1.set(0); //stopping intake
      conveyer1.set(0); //stopping conveyer
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_awaiting_shooter) {
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)){
        cargo_status = Robot_Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_being_shot) {
      state4_Timer.start();
      conveyer1.set(0.8); //running conveyer 
      if (state4_Timer.get() > 2.0) {
        conveyer1.set(0);
        shooter_motor1.set(0);
        state4_Timer.stop();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }  
  }

      /** 
   * This subroutine performs the robot operations manually
   *
   */
  public void manualIntake() {
    if(driver_joystick.getRawButton(5) == true){
      intake_motor1.set(-1);
    }
    else if(driver_joystick.getRawButton(5) == false){
      intake_motor1.set(driver_joystick.getRawAxis(2));
    }
    //Conveyor (positive inputs bring cargo in)
    if ((driver_joystick.getRawButton(4) == true) && (driver_joystick.getRawButton(3) == false)) {
      conveyer1.set(1);
    }
    else if((driver_joystick.getRawButton(4) == false) && driver_joystick.getRawButton(3) == true) {
      conveyer1.set(-1);
    }
    else {
      conveyer1.set(0);
    }
    //Shooter (positive inputs shoot cargo out)
    shooter_motor1.set(driver_joystick.getRawAxis(3)*0.9);
  }

  // This is is a custom type used to track the state of Cargo intake and shooting
  enum Robot_Cargo_State {
    Idle,    // This state means that the robot has no cargo in it and all intake/conveyor/shooter motors are off
    Cargo_being_intaked,     // This state means that a cargo is in the process of being intaked, but still in transit
    Cargo_awaiting_shooter,    // This state means that a cargo is in the robot and awaiting to be shot out
    Cargo_being_shot,    // This state means that the cargo is being shot out
    Ejecting,                 // This state means that the cargo is being ejected
    Error   // This is an error state or condition
  }

  enum Intake_Deployment_State {
    up, 
    down
  }
  
  enum Climber_State {
    start, 
    part1ClimbMiddleRung,
    part2ClimbMiddleRung,
    part1ClimbTopRung,
    part2ClimbTopRung,
    part3ClimbTopRung,
    part1ClimbTraversal,
    part2ClimbTraversal,
    part3ClimbTraversal,
    part4ClimbTraversal
  }

    /**
   * This method converts a target pitch angle into an estimated robot distance away from the target
   *
   * @param a2 The input angle representing the pitch of the target above the camera reticle, in degrees.
   * @return The predicted distance from the target from the shooter exit location, in inches.
   */
  double camAngletoDistance(double a2) {
    return ((goalHeight - pupilCameraHeight)/(Math.tan(Math.toRadians(cameraPitch + a2))) + pupilDistanceToShooter + goalRadius);
  }

      /**
   * This subroutine moves the intake from the Up to the Down position
   *
   */
  void moveIntakeUptoDown() {
    if (intake_status == Intake_Deployment_State.up) {
      intake_motor1.set(0);
      conveyer1.set(0);
      shooter_motor1.set(0);
      IntakeSolenoid.set(Value.kReverse);
      intake_status = Intake_Deployment_State.down;
    }
  }

      /**
   * This subroutine begins the process of climbing the middle rung
   *
   */
  void initiateMiddleRungClimb() {
    if ((Climber_status == Climber_State.start) && (intake_status == Intake_Deployment_State.up)) {
      climber_motor1.set(ControlMode.Position, 186000);
      if ((climber_motor1.getSelectedSensorPosition() >= 184000) && (shooter_motor1.getSelectedSensorPosition() <= 188000)){
        LeftClimberSolenoid1.set(Value.kReverse);
        RightClimberSolenoid1.set(Value.kReverse);
        Climber_status = Climber_State.part1ClimbMiddleRung;
      }
    }
  }

      /**
   * This subroutine completes the process of climbing the middle rung
   *
   */
  void finalizeMiddleRungClimb() {
    if (Climber_status == Climber_State.part1ClimbMiddleRung) {
      LeftClimberSolenoid1.set(Value.kForward);
      RightClimberSolenoid1.set(Value.kForward);
      Climber_status = Climber_State.part2ClimbMiddleRung;
    }
  }

      /**
   * This subroutine moves the intake from the Down to the Up position
   *
   */
  void moveIntakeDowntoUp() {
    if (intake_status == Intake_Deployment_State.down) {
      intake_motor1.set(0);
      conveyer1.set(0);
      shooter_motor1.set(0);
      IntakeSolenoid.set(Value.kForward);
      intake_status = Intake_Deployment_State.up;
    }  
  }
  void autoAim() {
    if (Math.abs(tx_angle) > 10.0){
      tarzan_robot.tankDrive(0.6*tx_angle/27, -0.6*tx_angle/27);
      
    }
    else if ((tx_angle >0.5) && (tx_angle < 10.0)){
      tarzan_robot.tankDrive(0.30, -0.30);
    }
    else if ((tx_angle > -10) && (tx_angle <-0.5)){
      tarzan_robot.tankDrive(-0.30, 0.30);
    }
    else{
      tarzan_robot.tankDrive(0, 0);
      cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
    

  }
  

}
void compressorTest() {
  if(driver_joystick.getRawButton(4) && (driver_joystick.getPOV() == 0)){
    LeftClimberSolenoid1.set(Value.kForward);
  }  
  else if(driver_joystick.getRawButton(2) && (driver_joystick.getPOV() == 0)){
    LeftClimberSolenoid1.set(Value.kReverse);
  }
  else if(driver_joystick.getRawButton(4) && (driver_joystick.getPOV() == 90)){
    RightClimberSolenoid1.set(Value.kForward);
  }
  else if(driver_joystick.getRawButton(2) && (driver_joystick.getPOV() == 90)){
    RightClimberSolenoid1.set(Value.kReverse);
  }
  else if(driver_joystick.getRawButton(4) && (driver_joystick.getPOV() == 180)){
    LeftClimberSolenoid2.set(Value.kForward);
  }
  else if(driver_joystick.getRawButton(2) && (driver_joystick.getPOV() == 180)){   
    LeftClimberSolenoid2.set(Value.kReverse);
  }
  else if(driver_joystick.getRawButton(4) && (driver_joystick.getPOV() == 270)){
    RightClimberSolenoid2.set(Value.kForward);
  }
  else if(driver_joystick.getRawButton(2) && (driver_joystick.getPOV() == 270)){
    RightClimberSolenoid2.set(Value.kReverse);
  }
  else if(driver_joystick.getRawButton(8) && driver_joystick.getRawButton(4)){
    IntakeSolenoid.set(Value.kForward);
  }  
  else if(driver_joystick.getRawButton(8) && driver_joystick.getRawButton(2)){
    IntakeSolenoid.set(Value.kReverse);
  }
}

void climberTest() {
  climber_motor1.set(copilot_joystick.getRawAxis(5));
}

void IntakeTest1() {
  if ((cargo_status == Robot_Cargo_State.Idle) && (copilot_joystick.getRawButton(6))) {
    cargo_status = Robot_Cargo_State.Cargo_being_intaked;
    state2_Timer.start();
  }
  else if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == true)) {
    //if (driver_joystick.getRawButton(6)){
    //  state2_Timer.start();
    //}
    
    

    intake_motor1.set(0.8); //running intake
    conveyer1.set(0.8); //running conveyer
    //shooter_motor1.set(1*0.8); //starting shooter at 80%
    //shooter_motor1.set(ControlMode.Velocity, 18000);
    if (state2_Timer.get() > 4.0) {
      intake_motor1.set(0);
      conveyer1.set(0);
      shooter_motor1.set(0);
      cargo_status = Robot_Cargo_State.Idle;
    }
  } 
  else if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == false)) {
    cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
    intake_motor1.set(0); //stopping intake
    conveyer1.set(0); //stopping conveyer
  }
}

void IntakeReverseTest() {
  if ((cargo_status == Robot_Cargo_State.Cargo_awaiting_shooter) && (copilot_joystick.getRawButton(3))) {
    state2_Timer.start();
    cargo_status = Robot_Cargo_State.Ejecting;
    intake_motor1.set(-0.8);
    conveyer1.set(-0.8);
  }
  else if (cargo_status == Robot_Cargo_State.Ejecting) {
    if (state2_Timer.get() > 4.0) {
      intake_motor1.set(0);
      conveyer1.set(0);
      state2_Timer.stop();
      cargo_status = Robot_Cargo_State.Idle;
    } 
  }
}
}
