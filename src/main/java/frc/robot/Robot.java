// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

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

  //DifferentialDrive tarzan_robot = new DifferentialDrive(driver_leftmotor1, driver_rightmotor1);

  private final DigitalInput conveyor_loc_1 = new DigitalInput(0);

  // Setup the pneumatics devices, 
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid IntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15); /* Make sure channel number associates with kReverse and Forward Ex: Channel 6 brings down (kReverse) and vice versa with channel 7*/
  DoubleSolenoid RightClimberSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 11); //Kforward is Retract and KReverse is Extend
  DoubleSolenoid LeftClimberSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 14);
  DoubleSolenoid RightClimberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 13);
  DoubleSolenoid LeftClimberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 12);

  private Cargo_State cargo_status = Cargo_State.Idle;
  private Intake_Deployment_State intake_status = Intake_Deployment_State.up;
  private Climber_State Climber_status = Climber_State.Start_1;
  private final Timer intake_Timer = new Timer();
  private final Timer shooter_Timer = new Timer();
  private final Timer climber_Timer = new Timer();
  private final Timer auto_Timer = new Timer();
  private double tx_angle;
  private double ty_angle = -1000.0; //target degrees above the center of the camera
  private double climberArmCommand = 0.0; //variable used to actively control the position of the climber arms; units are in counts at the climber1 motor shaft
  private double climberArmAdjust = 0.0;
  private double climberSetPoint = 0.0;

  private final double cameraPitch = 18.104; //degrees above horizon ||
  private final double pupilCameraHeight = 32.5; //inches above the ground ||
  private final double goalHeight = 104; //inches above the ground to the top of the goal
  private final double goalRadius = 26.7716535; //inches 
  private final double pupilDistanceToShooter = -6; //inches, in relation to distance from goal ||
  private final double desiredDistanceFromGoal = 155; //inches, distance from the shooter to the center of goal (114.75in - 24in) ||
  private final double minimum_climber_limit = -850000; // this is the absolute minimum safe climber arm rotation limit
  private final double maximum_climber_limit = 28500; // this is the absolute maximum safe climber arm rotation limit
  private final double climber_start_angle = 0;
  private final double climber_middle_angle = -175000;
  private final double climber_top_approach_angle = -450000;
  private final double climber_traversal_approach_angle = -800000;
  private final double climber_end_angle = -830000; 
  private final double climber_timeout = 1.0; // units in seconds
  
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

    driver_leftmotor1.setNeutralMode(NeutralMode.Coast);
    driver_leftmotor2.setNeutralMode(NeutralMode.Coast);
    driver_rightmotor1.setNeutralMode(NeutralMode.Coast);
    driver_rightmotor2.setNeutralMode(NeutralMode.Coast);

    //Mechanism Inversion settings 
    shooter_motor2.setInverted(true);
    driver_rightmotor1.setInverted(true);
    driver_rightmotor2.setInverted(true);
    climber_motor2.setInverted(true);
    conveyer1.setInverted(true);
    intake_motor1.setInverted(true);

    //Shooter, Driver, Climber Follow
    shooter_motor2.follow(shooter_motor1);
    driver_rightmotor2.follow(driver_rightmotor1);
    driver_leftmotor2.follow(driver_leftmotor1);
    climber_motor2.follow(climber_motor1);

    // Drive Base Velocity Control
		driver_rightmotor1.config_kP(0, 0.015, 30);
		driver_rightmotor2.config_kP(0, 0.015, 30);
    driver_leftmotor1.config_kP(0, 0.015, 30);
		driver_leftmotor2.config_kP(0, 0.015, 30);
    driver_rightmotor1.configClosedloopRamp(0.75);
    driver_rightmotor2.configClosedloopRamp(0.75);
    driver_leftmotor1.configClosedloopRamp(0.75);
    driver_leftmotor2.configClosedloopRamp(0.75);

    // Shooter Control Loop Settings
    shooter_motor1.configNeutralDeadband(0.001);
		shooter_motor1.config_kP(0, 0.010, 30);
		shooter_motor1.config_kI(0, 0.000, 30);
		shooter_motor1.config_kD(0, 0, 30);
    shooter_motor1.config_kF(0, 2048/22000, 30);
    shooter_motor2.configNeutralDeadband(0.001);
		shooter_motor2.config_kP(0, 0.015, 30);
		shooter_motor2.config_kI(0, 0.000, 30);
		shooter_motor2.config_kD(0, 0, 30);
    shooter_motor2.config_kF(0, 2048/22000, 30);

    climber_motor1.configNeutralDeadband(0.001);
		climber_motor1.config_kP(0, 0.010, 30);
		climber_motor1.config_kI(0, 0.000, 30);
		climber_motor1.config_kD(0, 0, 30);
    climber_motor2.configNeutralDeadband(0.001);
		climber_motor2.config_kP(0, 0.015, 30);
		climber_motor2.config_kI(0, 0.000, 30);
		climber_motor2.config_kD(0, 0, 30);

    climber_motor1.set(ControlMode.Position, 0.0);

    // configure limelight camera
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

    //Setup compressor controls for analog pressure transducer
    phCompressor.enableAnalog(110, 120);
    phCompressor.enabled();

    //Initializes Solenoids on position 'A'
    IntakeSolenoid.set(Value.kForward);
    RightClimberSolenoid1.set(Value.kForward);
    RightClimberSolenoid2.set(Value.kForward);
    LeftClimberSolenoid1.set(Value.kForward);
    LeftClimberSolenoid2.set(Value.kForward);

    climber_Timer.start();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("High Side Pressure", phCompressor.getPressure());
    SmartDashboard.putBoolean("Cargo Detected on Conveyor", conveyor_loc_1.get());
    SmartDashboard.putNumber("Climber Arm Angle relative to robot base", climberEncoder.get());
    SmartDashboard.putNumber("Distance to Goal", camAngletoDistance(ty_angle));
    SmartDashboard.putNumber("Climber 1 Encoder (counts)", climber_motor1.getSelectedSensorPosition());
    SmartDashboard.putString("Climber Status" , Climber_status.climber_sequence_name);
    SmartDashboard.putNumber("Climber Status Num" , Climber_status.climber_sequence_number);
    SmartDashboard.putString("Intake Status" , intake_status.intake_sequence_name);
    SmartDashboard.putString("Cargo Status" , cargo_status.cargo_sequence_name);
    SmartDashboard.putNumber("Climber Position Command", climberSetPoint);
    SmartDashboard.putNumber("Cargo Timer", intake_Timer.get());

    tx_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    
    updateClimberMotorPosition();
  }
    
  @Override
  public void autonomousInit() {
    auto_Timer.start();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    cargo_status = Cargo_State.Cargo_being_intaked;
  }

  @Override
  public void autonomousPeriodic() {
    moveIntakeUptoDown();
    if ((cargo_status == Cargo_State.Cargo_being_intaked) && (camAngletoDistance(ty_angle) <= desiredDistanceFromGoal)) {
      drive(10000, 10000);
      intake_motor1.set(0.8);
    }
    // Dan added the below, in case we have limelight problems, this will prevent us from crashing into wall and turn off the motors
    else if ((cargo_status == Cargo_State.Cargo_being_intaked) && (auto_Timer.get() > 10.0)) {
      auto_Timer.stop();
      drive(0, 0);
      intake_motor1.set(0.0);
      cargo_status = Cargo_State.Idle;
    }
    else if ((cargo_status == Cargo_State.Cargo_being_intaked) && (camAngletoDistance(ty_angle) > desiredDistanceFromGoal)) {
      auto_Timer.reset();
      auto_Timer.start();
      drive(0, 0);
      cargo_status = Cargo_State.Cargo_awaiting_shooter;
    }
    else if (cargo_status == Cargo_State.Cargo_awaiting_shooter) {
      shooter_motor1.set(0.9);
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)) {
        auto_Timer.reset();
        auto_Timer.start();
        cargo_status = Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Cargo_State.Cargo_being_shot) {
      conveyer1.set(0.8); //running conveyer 
      intake_motor1.set(0.8);
      if (auto_Timer.get() > 10.0){
        conveyer1.set(0);
        intake_motor1.set(0);
        shooter_motor1.set(0);
        auto_Timer.stop();
        cargo_status = Cargo_State.Idle;
      }
    }
  }

  @Override
  public void teleopInit() {
    cargo_status = Cargo_State.Idle;
  }

  @Override
  public void teleopPeriodic() {
    /**
    *          COPILOT JOYSTICK
    * Up D-PAD (pov 0) = Move Intake Up
    * Down D-PAD (pov 180) = Move Intake Down
    * Left D-Pad = Go to Previous Climb State
    * Right D-Pad = Go to Next Climb State
    * A button (button 1) = Run AutoAim function while holding
    * Right Bumper (button 6) = Perform Autoshoot function while holding (completes after 1.5 seconds)
    * Left Bumper (button 5) = Perform Autointake function while holding (after 4 seconds of no pressing, it cancels)  
    *           PILOT JOYSTICK
    * Left Stick Up/Down (raw axis 1) = Move Robot left side
    * Right Stick Up/Down (raw axis 5) = Move Robot right side  
    **/

    // HANDLE INTAKE POSITION STATE CHANGES
    if (copilot_joystick.getPOV()== 0) { //Up on the D-Pad
      moveIntakeDowntoUp();
    }
    else if (copilot_joystick.getPOV()== 180) { //Down on the D-PAD
      moveIntakeUptoDown();
    }

    // HANDLE INTAKE AND SHOOTING STATES
    processCargoState(copilot_joystick.getRawButton(5), copilot_joystick.getRawButton(6));

    // HANDLE CLIMBER STATE CHANGES with programable min delay between each change
    if ((copilot_joystick.getPOV() == 90) && (climber_Timer.get() > climber_timeout)) {
      nextClimberState();      
    }
    else if ((copilot_joystick.getPOV() == 270) && (climber_Timer.get() > climber_timeout)) {
      previousClimberState();      
    }
    processClimberState(copilot_joystick.getRawAxis(3), copilot_joystick.getRawAxis(2));

    // HANDLE DRIVE TRAIN COMMANDS
    if (copilot_joystick.getRawButton(1)){
      autoAim();
    }
    else {
      drive(-40000*driver_joystick.getRawAxis(1), -40000*driver_joystick.getRawAxis(5));
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    cargo_status = Cargo_State.Idle;
  }

  @Override
  public void testPeriodic() {
  }

  enum Cargo_State {
    Idle("Idle", 1),    // This state means that the robot has no cargo in it and all intake/conveyor/shooter motors are off
    Cargo_being_intaked("Being Intaked", 2),     // This state means that a cargo is in the process of being intaked, but still in transit
    Cargo_awaiting_shooter("Awaiting Shooter", 3),    // This state means that a cargo is in the robot and awaiting to be shot out
    Cargo_being_shot("Being Shot", 4);    // This state means that the cargo is being shot out

    public String cargo_sequence_name;
    public int cargo_sequence_number;

    Cargo_State(String Name, int Number) {
      this.cargo_sequence_name = Name;
      this.cargo_sequence_number = Number;
    }
  }

  enum Intake_Deployment_State {
    up("Up", 1), 
    down("Down", 2);

    public String intake_sequence_name;
    public int intake_sequence_number;

    Intake_Deployment_State(String Name, int Number) {
      this.intake_sequence_name = Name;
      this.intake_sequence_number = Number;
    }
  }
  
  enum Climber_State {
    Start_1("Start", 1),
    BeginMiddleClimb_2("Begin Middle Climb", 2),
    ExtendMiddleClimb_3("Extend Middle Climb",3),
    FinishMiddleClimb_4("Finish Middle Climb", 4),
    BeginTopClimb_5("Begin Top Climb", 5),
    ExtendTopClimbArms_6("Extend Top Climb Arms", 6),
    RetractTopRung_7("Retract Top Rung", 7),
    ExtendMidRung_8("Extend Mid Rung", 8),
    RetractMidRung_9("Retract after Top Rung", 9),
    BeginTraversalClimb_10("Begin Traversal Climb", 10),
    ExtendTraversalClimbArms_11("Extend Traversal Climb Arms", 11),
    RetractTraversalRung_12("Retract Traversal Rung", 12),
    ExtendTopRung_13("Extend Top Rung", 13),
    RetractTopRung_14("Retract after Traversal Rung",14),
    End_15("End", 15);

    public String climber_sequence_name;
    public int climber_sequence_number;

    Climber_State(String Name, int Number) {
      this.climber_sequence_name = Name;
      this.climber_sequence_number = Number;
    }
  }

 /**
   * This method changes the desired climber state to the next one
   *
   */
  void nextClimberState() {
    if (Climber_status.climber_sequence_number < 15) {
      if (Climber_status.climber_sequence_number == 1) {
        Climber_status = Climber_State.BeginMiddleClimb_2;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 2) {
        Climber_status = Climber_State.ExtendMiddleClimb_3;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 3) {
        Climber_status = Climber_State.FinishMiddleClimb_4;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 4) {
        Climber_status = Climber_State.BeginTopClimb_5;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 5) {
        Climber_status = Climber_State.ExtendTopClimbArms_6;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 6) {
        Climber_status = Climber_State.RetractTopRung_7;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 7) {
        Climber_status = Climber_State.ExtendMidRung_8;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 8) {
        Climber_status = Climber_State.RetractMidRung_9;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 9) {
        Climber_status = Climber_State.BeginTraversalClimb_10;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 10) {
        Climber_status = Climber_State.ExtendTraversalClimbArms_11;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 11) {
        Climber_status = Climber_State.RetractTraversalRung_12;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 12) {
        Climber_status = Climber_State.ExtendTopRung_13;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 13) {
        Climber_status = Climber_State.RetractTopRung_14;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 14) {
        Climber_status = Climber_State.End_15;
        climber_Timer.reset();
      }
    }
  }

   /**
   * This method changes the desired climber state to the previous one
   *
   */
  void previousClimberState() {
    if (Climber_status.climber_sequence_number > 1) {
      if  (Climber_status.climber_sequence_number == 2) {
        Climber_status = Climber_State.Start_1;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 3) {
        Climber_status = Climber_State.BeginMiddleClimb_2;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 4) {
        Climber_status = Climber_State.ExtendMiddleClimb_3;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 5) {
        Climber_status = Climber_State.FinishMiddleClimb_4;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 6) {
        Climber_status = Climber_State.BeginTopClimb_5;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 7) {
        Climber_status = Climber_State.ExtendTopClimbArms_6;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 8) {
        Climber_status = Climber_State.RetractTopRung_7;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 9) {
        Climber_status = Climber_State.ExtendMidRung_8;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 10) {
        Climber_status = Climber_State.RetractMidRung_9;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 11) {
        Climber_status = Climber_State.BeginTraversalClimb_10;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 12) {
        Climber_status = Climber_State.ExtendTraversalClimbArms_11;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 13) {
        Climber_status = Climber_State.RetractTraversalRung_12;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 14) {
        Climber_status = Climber_State.ExtendTopRung_13;
        climber_Timer.reset();
      }
      else if  (Climber_status.climber_sequence_number == 15) {
        Climber_status = Climber_State.RetractTopRung_14;
        climber_Timer.reset();
      }
    }
  }

  /**
   * This method converts a target pitch angle into an estimated robot distance away from the target
   *
   * @param a2 The input angle representing the pitch of the target above the camera reticle, in degrees.
   * @return The predicted distance from the target from the shooter exit location, in inches.
   */
  double camAngletoDistance(double a2) {
    if (a2 < 1){
      return 0;
    }
    else {
      return ((goalHeight - pupilCameraHeight)/(Math.tan(Math.toRadians(cameraPitch + a2))) + pupilDistanceToShooter + goalRadius);
    }
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

  /**
   * This method handles all of the climber states and related controls
   * @param climb_forward The controller input (trigger) that moves the climber forward manually.
   * @param climb_backward The controller input (trigger) that moves the climber backward manually.
   */
  void processClimberState(double climb_forward, double climb_backward) {
    if (Climber_status == Climber_State.Start_1) {
      climberSetPoint = climber_start_angle;
    }
    else if (Climber_status == Climber_State.BeginMiddleClimb_2) {
      climberArmCommand = climber_middle_angle;
    }
    else if (Climber_status == Climber_State.ExtendMiddleClimb_3) {
      LeftClimberSolenoid2.set(Value.kReverse);
      RightClimberSolenoid2.set(Value.kReverse);
    }
    else if (Climber_status == Climber_State.FinishMiddleClimb_4) {
      LeftClimberSolenoid2.set(Value.kForward);
      RightClimberSolenoid2.set(Value.kForward);
    }
    else if (Climber_status == Climber_State.BeginTopClimb_5) {
      climberSetPoint = climber_top_approach_angle;
    }
    else if (Climber_status == Climber_State.ExtendTopClimbArms_6) {
      LeftClimberSolenoid1.set(Value.kReverse);
      RightClimberSolenoid1.set(Value.kReverse);
    }
    else if (Climber_status == Climber_State.RetractTopRung_7) {
      LeftClimberSolenoid1.set(Value.kForward);
      RightClimberSolenoid1.set(Value.kForward);
    }
    else if (Climber_status == Climber_State.ExtendMidRung_8) {
      LeftClimberSolenoid2.set(Value.kReverse);
      RightClimberSolenoid2.set(Value.kReverse);
    }
    else if (Climber_status == Climber_State.RetractMidRung_9) {
      LeftClimberSolenoid2.set(Value.kForward);
      RightClimberSolenoid2.set(Value.kForward);
    }
    else if (Climber_status == Climber_State.BeginTraversalClimb_10) {
      climberSetPoint = climber_traversal_approach_angle;
    }
    else if (Climber_status == Climber_State.ExtendTraversalClimbArms_11) {
      LeftClimberSolenoid2.set(Value.kReverse);
      RightClimberSolenoid2.set(Value.kReverse);
    }
    else if (Climber_status == Climber_State.RetractTraversalRung_12) {
      LeftClimberSolenoid2.set(Value.kForward);
      RightClimberSolenoid2.set(Value.kForward);
    }
    else if (Climber_status == Climber_State.ExtendTopRung_13) {
      LeftClimberSolenoid1.set(Value.kReverse);
      RightClimberSolenoid1.set(Value.kReverse);
    }
    else if (Climber_status == Climber_State.RetractTopRung_14) {
      LeftClimberSolenoid1.set(Value.kForward);
      RightClimberSolenoid1.set(Value.kForward);
    }
    else if (Climber_status == Climber_State.End_15) {
      climberSetPoint = climber_end_angle;
    }

    // the following allows the copilot to move the climber arms together either forward or background by holding either
    // of the shoulder trigger buttons. The harder you press, the further it changes. Every 20 msec, at full press, the
    // climber would change ~600 counts or ~30000 counts (~15 degrees) per second.
    climberArmAdjust = climberArmAdjust + 600*climb_forward - 600*climb_backward;

    //      CLIMBER ARM COMMAND CLAMPING LOGIC
    // the following block of code ensures that the climber arms can never be driven beyond safe operating ranges
    // if the driver attempts to command beyond the safe limit, only the min/max safe command is applied
    if ((climberSetPoint + climberArmAdjust) < minimum_climber_limit) {
      climberArmCommand = minimum_climber_limit;
    }
    else if ((climberSetPoint + climberArmCommand) > maximum_climber_limit) {
      climberArmCommand = maximum_climber_limit;
    }
    else {
      climberSetPoint = climberArmCommand + climberArmAdjust;
    }
  climber_motor1.set(ControlMode.Position, climberSetPoint);
  }  

  /**
   * This method handles all of the climber states and related controls
   * @param joystick_intake_command The controller input (button) that commands intake action.
   * @param joystick_shoot_command The controller input (button) that commands shoot action.
   */
  void processCargoState(boolean joystick_intake_command, boolean joystick_shoot_command) {
    if ((cargo_status == Cargo_State.Idle) && joystick_intake_command) { // starts cargo intake on single press
      cargo_status = Cargo_State.Cargo_being_intaked;
      intake_Timer.reset();
      intake_Timer.start();      
    }
    else if (cargo_status == Cargo_State.Cargo_being_intaked) {
      if (conveyor_loc_1.get() == true) { // if cargo is not seen, keep attempting to intake
        intake_motor1.set(0.8);
        conveyer1.set(0.8);
      }
      else if (conveyor_loc_1.get() == false) { // if cargo is seen, stop the intake are start shooting process
        intake_motor1.set(0);
        conveyer1.set(0);
        intake_Timer.stop();
        cargo_status = Cargo_State.Cargo_awaiting_shooter;
      }

      if (intake_Timer.get() > 4.0) { // if it has been 4 seconds and no cargo seen, then halt and clear state
        intake_motor1.set(0);
        conveyer1.set(0);
        intake_Timer.stop();
        cargo_status = Cargo_State.Idle;
      }     
    }
    else if (cargo_status == Cargo_State.Cargo_awaiting_shooter) {
      if (joystick_intake_command) { // This allows you to intake another cargo after one has already completely entered
        intake_motor1.set(0.8);
      }
      else {
        intake_motor1.set(0);
      }

      if (joystick_shoot_command) { // This only runs the shooter while the button is held
        shooter_motor1.set(ControlMode.Velocity, 18000);
      }
      else {
        shooter_motor1.set(ControlMode.Velocity, 0);
      }

      if (shooter_motor1.getSelectedSensorVelocity() >= 17500) { // only starts the cargo firing process after speed is achieved
        shooter_Timer.reset();
        shooter_Timer.start();      
        cargo_status = Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Cargo_State.Cargo_being_shot) {
      intake_motor1.set(0);
      conveyer1.set(0.8);
      if (shooter_Timer.get() > 1.5){ // this ensures the shooter is cleared of cargo
        conveyer1.set(0);
        shooter_motor1.set(0);
        shooter_Timer.stop();
        cargo_status = Cargo_State.Idle;
      }
    }
  }

  /**
   * This subroutine moves the drive train to face the hub
   *
   */
  void autoAim() {
    if (Math.abs(tx_angle) > 10.0){
      //tarzan_robot.tankDrive(0.6*tx_angle/27, -0.6*tx_angle/27);
      drive(750*tx_angle,-750*tx_angle);
    }
    else if ((tx_angle >0.5) && (tx_angle < 10.0)){
      //tarzan_robot.tankDrive(0.30, -0.30);
      drive(3000, -3000);
    }
    else if ((tx_angle > -10) && (tx_angle <-0.5)){
      //tarzan_robot.tankDrive(-0.30, 0.30);
      drive(-3000, 3000);
    }
    else{
      //tarzan_robot.tankDrive(0, 0);
      //cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;  //How and why did this get here?
      drive(0, 0);
    }
  }

    /**
   * This subroutine updates the climber relative encoder value to sync up with the shaft absolute encoder feedback
   *
   */
  void updateClimberMotorPosition() {
    // climberEncoder.get() returns the true climber arm shaft angle. * by 360 converts it to degrees. Subtracting
    // 117.36 is based on measurements that offset the angle so that it reads 0 when the climber is horizontal
    // and in the starting configuration. Finally, we * 2044.4 (need to verify) to convert degrees at the climber motor
    // shaft into counts.  This will adjust the position of the climber motor shaft so that it matches what the actual
    // position of the climber arm shaft and effectively corrects for any slip or incorrect positioning before the
    // robot is turned on (recall that the arm is supposed to start in a horizontal position.

    // Here is an example. If the robot were mistakenly started with the arms vertical, the climberEncoder.get() would
    // return a 0.576 (based on test measurements). This would get converted to ~184000 and overwrite the climber motor 1
    // position from 0 to 184000, thereby re-aligning the two. If slip between the two were to occur during the match,
    // calling this function periodically would also correct for those misalignments as well.
    climber_motor1.setSelectedSensorPosition(1777.78*(climberEncoder.get()*360 - 117.36));
  }

  /**
   * This method controls the drive train using a basic velocity tank control mode.
   * @param leftControl The controller input (stick) that commands the left side.
   * @param rightControl The controller input (stick) that commands the right side.
   */
  void drive(double leftControl, double rightControl) {
    driver_leftmotor1.set(ControlMode.Velocity, leftControl);
    driver_rightmotor1.set(ControlMode.Velocity, rightControl);
  }
}