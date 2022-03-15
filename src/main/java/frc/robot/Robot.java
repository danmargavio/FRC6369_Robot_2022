// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class Robot extends TimedRobot {
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
  private final Joystick driver_joystick = new Joystick(0);
  private final Joystick copilot_joystick = new Joystick(1);
  DifferentialDrive tarzan_robot = new DifferentialDrive(driver_leftmotor1, driver_rightmotor1);
  private final DigitalInput conveyor_loc_1 = new DigitalInput(0);

  // Setup the pneumatics devices, 
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid IntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15); /* Make sure channel number associates with kReverse and Forward Ex: Channel 6 brings down (kReverse) and vice versa with channel 7*/
  DoubleSolenoid RightClimberSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 11); //Kforward is Retract and KReverse is Extend
  DoubleSolenoid LeftClimberSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 14);
  DoubleSolenoid RightClimberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 13);
  DoubleSolenoid LeftClimberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 12);

  // Setup the ADIS16448 IMU
  public static final ADIS16448_IMU gyro = new ADIS16448_IMU();

  private Robot_Cargo_State cargo_status = Robot_Cargo_State.Idle;
  private Intake_Deployment_State intake_status = Intake_Deployment_State.up;
  private Climber_State Climber_status = Climber_State.start;
  private final Timer state4_Timer = new Timer();
  private final Timer state2_Timer = new Timer();
  private double tx_angle;
  private double ty_angle = -1000.0; //target degrees above the center of the camera
  private double climberArmCommand = 0.0; //variable used to actively control the position of the climber arms; units are in counts at the climber1 motor shaft

  private final double cameraPitch = 18.104; //degrees above horizon ||
  private final double pupilCameraHeight = 32.5; //inches above the ground ||
  private final double goalHeight = 104; //inches above the ground to the top of the goal
  private final double goalRadius = 26.7716535; //inches 
  private final double pupilDistanceToShooter = -6; //inches, in relation to distance from goal ||
  private final double desiredDistanceFromGoal = 120; //inches, distance from the shooter to the center of goal (114.75in - 24in) ||
  private final double minimum_climber_limit = -850000; // this is the absolute minimum safe climber arm rotation limit
  private final double maximum_climber_limit = 25000; // this is the absolute maximum safe climber arm rotation limit
  
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

    //Shooter, Driver, Climber Follow
    shooter_motor2.follow(shooter_motor1);
    driver_rightmotor2.follow(driver_rightmotor1);
    driver_leftmotor2.follow(driver_leftmotor1);
    climber_motor2.follow(climber_motor1);

    // Control Loop Settings
    shooter_motor1.configNeutralDeadband(0.001);
		shooter_motor1.config_kP(0, 0.015, 30);
		shooter_motor1.config_kI(0, 0.000, 30);
		shooter_motor1.config_kD(0, 0, 30);
    shooter_motor1.config_kF(0, 2048/22000, 30);
    climber_motor1.configNeutralDeadband(0.001);
		climber_motor1.config_kP(0, 0.015, 30);
		climber_motor1.config_kI(0, 0.000, 30);
		climber_motor1.config_kD(0, 0, 30);
    climber_motor1.set(ControlMode.Position, 0.0);

    // configure limelight camera
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

    //Setup compressor controls for analog pressure transducer
    phCompressor.enableAnalog(90, 120);
    phCompressor.enabled();

    //Initializes Solenoids on position 'A'
    IntakeSolenoid.set(Value.kForward);
    RightClimberSolenoid1.set(Value.kForward);
    RightClimberSolenoid2.set(Value.kForward);
    LeftClimberSolenoid1.set(Value.kForward);
    LeftClimberSolenoid2.set(Value.kForward);    
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("PSI", phCompressor.getPressure());
    SmartDashboard.putBoolean("Ball In", conveyor_loc_1.get());
    SmartDashboard.putNumber("Climber Arm Angle relative to robot base", climberEncoder.get());
    SmartDashboard.putNumber("Climber Arm Angle relative to ground", climberEncoder.get()*360 - 117.36);    // Convert encoder to degrees than subtract offet so it reads 0.0 deg when horizontal
    SmartDashboard.putNumber("distance", camAngletoDistance(ty_angle));
    SmartDashboard.putNumber("Climber 1 Encoder (counts)", climber_motor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Robot Base Pitch", gyro.getGyroAngleY()); // this is robot pitch (front to back)
    tx_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }
    
  @Override
  public void autonomousInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
    cargo_status = Robot_Cargo_State.Cargo_being_intaked;
  }

  @Override
  public void autonomousPeriodic() {
    limelightCheck();
    moveIntakeUptoDown();

    if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (camAngletoDistance(ty_angle) <= desiredDistanceFromGoal)) {
      tarzan_robot.tankDrive(0.4, 0.4);
      intake_motor1.set(0.8);
    }
    else if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (camAngletoDistance(ty_angle) > desiredDistanceFromGoal)) {
      /*if (Math.abs(tx_angle) > 0.5){
        tarzan_robot.tankDrive(-1*tx_angle, 1*tx_angle);
      }*/
      //autoAim();
      state4_Timer.start();
      cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_awaiting_shooter) {
      shooter_motor1.set(0.9);
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)) {
        state4_Timer.start();
        cargo_status = Robot_Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_being_shot) {
      conveyer1.set(0.8); //running conveyer 
      if (state4_Timer.get() > 5.0){
        conveyer1.set(0);
        intake_motor1.set(0);
        shooter_motor1.set(0);
        state4_Timer.stop();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }
  }

  @Override
  public void teleopInit() {
    cargo_status = Robot_Cargo_State.Idle;
  }

  @Override
  public void teleopPeriodic() {
    /**
    *           COPILOT JOYSTICK
    * Back (button 7) AND X (button 3) = Prepare for Middle Rung Climb
    * Back (button 7) AND A (button 1) = Perform Middle Rung Climb
    *           PILOT JOYSTICK
    * Left Stick Up/Down (raw axis 1) = Move Robot left side
    * Right Stick Up/Down (raw axis 5) = Move Robot right side
    * Back (button 7) AND X (button 3) = Move Intake Up
    * Back (button 7) AND A (button 1) = Move Intake Down
    * Left Bumper (button 5) = Perform Autointake function while holding (after 4 seconds of no pressing, it cancels)
    * B (button 2) = Run AutoAim function while holding
    * X (button 3) = Perform Autoshoot function while holding (completes after 1.5 seconds)
    
    **/
    limelightCheck(); 
    manualIntake();
    autoIntake(); // currently replaces manualIntake();
    autoShoot(); //shoot

    if (copilot_joystick.getPOV() != 270){
    	climberTest2();
    }
    if (driver_joystick.getRawButton(7) && driver_joystick.getRawButton(3)){
    moveIntakeDowntoUp();
    }
    if (driver_joystick.getRawButton(7) && driver_joystick.getRawButton(1)){
    moveIntakeUptoDown();
    }
    //If Driver is controlling, don't auto aim, but if driver presses button they are forced to switch to auto aiming
    if (driver_joystick.getRawButton(2)){
      autoAim();
    }
    else{
      nonlinearDrive(driver_joystick.getRawAxis(1), driver_joystick.getRawAxis(5));
    }
    //Intake (positive inputs intake a cargo)
    if (intake_status == Intake_Deployment_State.down){
      if (copilot_joystick.getRawButton(7) && copilot_joystick.getRawButton(3)){
        initiateMiddleRungClimb();
      }
      if (copilot_joystick.getRawButton(7) && copilot_joystick.getRawButton(1)){
        finalizeMiddleRungClimb();
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    cargo_status = Robot_Cargo_State.Idle;
    updateClimberMotorPosition(); // Test it first without this, then add it in
  }

  @Override
  public void testPeriodic() {
    tarzan_robot.tankDrive(-0.5*driver_joystick.getRawAxis(1), -0.5*driver_joystick.getRawAxis(5));
    //         COPILOT JOYSTICK
    // Back Button (raw button 7) = Move Intake from Down to Up.
    if (copilot_joystick.getRawButton(7)){
      moveIntakeDowntoUp();
    }
    //         COPILOT JOYSTICK
    // Start Button (raw button 8) = Move Intake from Up to Down.
    if (copilot_joystick.getRawButton(8)){
      moveIntakeUptoDown();
    }
    climberTest2();
    updateClimberMotorPosition(); // Test it first without this, then add it in
  }

    /**
   * This subroutine performs the semi-autonomous intake and shooting process
   *
   */
  public void autoIntake() {
    if((cargo_status == Robot_Cargo_State.Idle) && (driver_joystick.getRawButton(5) == true)){
      cargo_status = Robot_Cargo_State.Cargo_being_intaked;
      state2_Timer.reset();
      state2_Timer.start();
    }

    if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == true)) {
      if (driver_joystick.getRawButton(5) == true){
        state2_Timer.reset();
      }
      intake_motor1.set(0.8); //running intake
      conveyer1.set(0.8); //running conveyer
      if (state2_Timer.get() > 4.0) {
        intake_motor1.set(0);
        conveyer1.set(0);
        state2_Timer.stop();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }
    else if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == false)) {
      cargo_status = Robot_Cargo_State.Cargo_loaded;
      state2_Timer.stop();
      state2_Timer.reset();
      intake_motor1.set(0); //stopping intake
      conveyer1.set(0); //stopping conveyer
    }
    else if ((cargo_status == Robot_Cargo_State.Cargo_loaded) && (conveyor_loc_1.get() == false)) {
      if (driver_joystick.getRawButton(5) == true){
        state2_Timer.reset();
        state2_Timer.start();
      }
      intake_motor1.set(0.8); //running intake
      if (state2_Timer.get() > 2.0) {
        intake_motor1.set(0);
        state2_Timer.stop();
        cargo_status = Robot_Cargo_State.Cargo_double_loaded;
      }
    }
  }

  public void autoShoot() {
    if ((cargo_status == Robot_Cargo_State.Cargo_loaded) && (driver_joystick.getRawButton(3))) {
      shooter_motor1.set(0.9);
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)) {
        state4_Timer.reset();
        state4_Timer.start();      
        cargo_status = Robot_Cargo_State.Cargo_single_shot;
      }
    }
    else if ((cargo_status == Robot_Cargo_State.Cargo_double_loaded) && (driver_joystick.getRawButton(3))) {
      shooter_motor1.set(0.9);
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)) {
        state4_Timer.reset();
        state4_Timer.start();      
        cargo_status = Robot_Cargo_State.Cargo_double_shot;
      }
    }
    else if (((cargo_status == Robot_Cargo_State.Cargo_single_shot) || (cargo_status == Robot_Cargo_State.Cargo_double_shot)) && (driver_joystick.getRawButton(3) == false)) {
      shooter_motor1.set(0.0);
    }
    else if ((cargo_status == Robot_Cargo_State.Cargo_single_shot)) {
      conveyer1.set(0.8); //running conveyer 
      if (state4_Timer.get() > 1.5){
        conveyer1.set(0);
        shooter_motor1.set(0);
        state4_Timer.stop();
        state4_Timer.reset();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }
    else if ((cargo_status == Robot_Cargo_State.Cargo_double_shot)) {
      conveyer1.set(0.8); //running conveyer 
      intake_motor1.set(0.8);
      if (state4_Timer.get() > 4){
        conveyer1.set(0);
        shooter_motor1.set(0);
        intake_motor1.set(0);
        state4_Timer.stop();
        state4_Timer.reset();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }
  }

   /** 
   * This subroutine performs the robot operations manually (MUST HOLD LEFT POV TO USE THESE)
   * 
   *         COPILOT JOYSTICK
   * Right Trigger (raw axis 3) = Rotate shooter at 90% output.
   * Left Trigger (raw axis 2) = Intake Motor Forward.
   * Left Bumper (button 5) = Intake Motor Reverse.
   * Y (button 4) = Conveyor Motor Forward.
   * X (button 3) = Conveyor Motor Reverse.
   * 
   */
  public void manualIntake() {
    if (copilot_joystick.getPOV() == 270){
	    cargo_status = Robot_Cargo_State.Idle;    // You forgot to include this; this is how you reset the robot in an emergency
      if(copilot_joystick.getRawButton(1) == true){
        intake_motor1.set(-1);
      }
      else if(copilot_joystick.getRawButton(3) == true){
        intake_motor1.set(1);
      }
      else {
        intake_motor1.set(0);
      }
      //Conveyor (positive inputs bring cargo in)
      if ((copilot_joystick.getRawButton(4) == true)) {
        conveyer1.set(1);
      }
      else if((copilot_joystick.getRawButton(2) == true)) {
        conveyer1.set(-1);
      }
      else {
        conveyer1.set(0);
      }
      //Shooter (positive inputs shoot cargo out)
      shooter_motor1.set(copilot_joystick.getRawAxis(3)*0.9);
    }
  }

  // This is is a custom type used to track the state of Cargo intake and shooting
  enum Robot_Cargo_State {
    Idle,    // This state means that the robot has no cargo in it and all intake/conveyor/shooter motors are off
    Cargo_being_intaked,     // This state means that a cargo is in the process of being intaked, but still in transit
    Cargo_awaiting_shooter,    // This state means that a cargo is in the robot and awaiting to be shot out
    Cargo_being_shot,    // This state means that the cargo is being shot out
    Cargo_loaded,
    Cargo_single_shot, 
    Cargo_double_shot,
    Cargo_double_loaded,
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
      LeftClimberSolenoid1.set(Value.kForward); //right1 solenoid retract
      RightClimberSolenoid1.set(Value.kForward); //left1 solenoid retract
    }  
    else if(driver_joystick.getRawButton(2) && (driver_joystick.getPOV() == 0)){
      LeftClimberSolenoid1.set(Value.kReverse); //right1 solenoid extend
      RightClimberSolenoid1.set(Value.kReverse); //left1 solenoid extend
    }
    else if(driver_joystick.getRawButton(4) && (driver_joystick.getPOV() == 180)){
      LeftClimberSolenoid2.set(Value.kForward); //right2 solenoid retract
      RightClimberSolenoid2.set(Value.kForward); //left2 solenoid retract
    }
    else if(driver_joystick.getRawButton(2) && (driver_joystick.getPOV() == 180)){   
      LeftClimberSolenoid2.set(Value.kReverse); //right2 solenoid extend
      RightClimberSolenoid2.set(Value.kReverse); //left2 solenoid extend
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
      intake_motor1.set(0.8); //running intake
      conveyer1.set(0.8); //running conveyer
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

  void nonlinearDrive(double x, double y) { //x = driver_joystick.getRawAxis(1) && y = driver_joystick.getRawAxis(5)
    tarzan_robot.tankDrive(-0.75*Math.signum(x)*Math.pow(x,2), -0.75*Math.signum(y)*Math.pow(y,2));
  }

  /**
     * This method provides controls to the copilot to manually manipulate the climber in a controlled way.
     *
     *        COPILOT JOYSTICK
     * Right Trigger (raw axis 3) = Rotate the climber forwards at 5 degrees per second.
     * Left Trigger (raw axis 2) = Rotate the climber backwards at 5 degrees per second. 
     * Y (button 4) = Left 1 and Right 1 Extend.
     * B (button 2) = Left 1 and Right 1 Retract.
     * X (button 3) = Left 2 and Right 2 Extend.
     * A (button 1) = Left 2 and Right 2 Retract.
     *
     */
  void climberTest2() {
    if ((Climber_status == Climber_State.start) && (intake_status == Intake_Deployment_State.up)) {
      // the following allows the copilot to move the climber arms together either forward or background by holding either
      // of the shoulder trigger buttons. The harder you press, the further it changes. Every 20 msec, at full press, the
      // climber would change ~200 counts or ~10000 counts (5 degrees) per second.
      climberArmCommand = climberArmCommand + 450*copilot_joystick.getRawAxis(3) - 450*copilot_joystick.getRawAxis(2);

      //      CLIMBER ARM COMMAND CLAMPING LOGIC
      // the following block of code ensures that the climber arms can never be driven beyond safe operating ranges
      // if the driver attempts to command beyond the safe limit, only the min/max safe command is applied
      if (climberArmCommand < minimum_climber_limit) {
        climberArmCommand = minimum_climber_limit;
      }
      else if (climberArmCommand > maximum_climber_limit) {
        climberArmCommand = maximum_climber_limit;
      }

      // this actually applies the climber arm command to the control loop
      climber_motor1.set(ControlMode.Position, climberArmCommand);

      // this allows the copilot to command the climber arm pistons (a pair at a time) by pressing the four face buttons
      // Y (button 4) = Left 1 and Right 1 Extend
      // B (button 2) = Left 1 and Right 1 Retract
      // X (button 3) = Left 2 and Right 2 Extend
      // A (button 1) = Left 2 and Right 2 Retract
        if (copilot_joystick.getRawButton(4)) {
        LeftClimberSolenoid1.set(Value.kReverse);
        RightClimberSolenoid1.set(Value.kReverse);
      }
      if (copilot_joystick.getRawButton(2)) {
        LeftClimberSolenoid1.set(Value.kForward);
        RightClimberSolenoid1.set(Value.kForward);
      }
      if (copilot_joystick.getRawButton(3)) {
        LeftClimberSolenoid2.set(Value.kReverse);
        RightClimberSolenoid2.set(Value.kReverse);
      }
      if (copilot_joystick.getRawButton(1)) {
        LeftClimberSolenoid2.set(Value.kForward);
        RightClimberSolenoid2.set(Value.kForward);
      }
    }
  }

  /**
     * This method updates the relative position sense of the climber 1 motor to match the true position at the
     * climber arm shaft
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
  void limelightCheck() {
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0) != 4){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
    }
  }
}