/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.Math;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private enum Height{
    CARGO1, CARGO2, CARGO3, ROCKET1, ROCKET2, ROCKET3
  }

  private Height currentPosition;

  private final double CARGO_BOTTOM  = 12 + 7/12; //inches
  private final double ROCKET_BOTTOM = 27.5;
  private final double HEIGHT_STEP = 24 + 4/12; 
  private final double ROCKET_CARGO_STEP = ROCKET_BOTTOM - CARGO_BOTTOM;

  //private boolean pivotedUp;

  //Controller and drive
  private static XboxController controller;
  private static DifferentialDrive driveTrain;

  //Motor control - left
  private static Spark m_left;
  //private static Spark m_leftRear;
  //private static SpeedControllerGroup m_left;

  //Motor control - right
  private static Spark m_right;
  //private static Spark m_rightRear;
  //private static SpeedControllerGroup m_right;

  //Ball intake
  private static Spark m_intakeLeft;
  private static Spark m_intakeRight;

  //Arm control
  private static Spark m_armLeft;
  private static Spark m_armRight;
  private static Encoder armEncoder;

  //Manipulator pivot
  private static Spark m_pivot;
  private static DigitalInput pivotLimitUp;  //Upper limit
  private static DigitalInput pivotLimitDown; //Lower limit
  private boolean pivotedUp;

  //Legs to the lift the robot
  private static Spark m_legL;
  private static Spark m_legR;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //double trigger_valueL;
    //double trigger_valueR;
    //currentPosition = Height.CARGO1;

    controller = new XboxController(0); //port is the port of the driver's station
    CameraServer.getInstance().addAxisCamera("10.67.50.87"); //Add camera
    CameraServer.getInstance().startAutomaticCapture();     //Start camera
    
    /* 3  1  Motor number map
     * 4  2  for drive train */
    //Motors are grouped left and right
    m_left  = new Spark(0);
    m_right = new Spark(1);
    //m_left  = new SpeedControllerGroup(m_leftFront, m_leftRear);   //Left motor side
    //m_right = new SpeedControllerGroup(m_rightFront, m_rightRear); //Right motor side
    driveTrain = new DifferentialDrive(m_left, m_right);

    //Ball intake motor control
    m_intakeLeft  = new Spark(2);
    m_intakeRight = new Spark(3);

    //Arm control
    //m_arm = new Spark(7);
    m_armLeft  = new Spark(4);
    m_armRight = new Spark(5);
    armEncoder = new Encoder(1, 2);
    armEncoder.setDistancePerPulse((Math.PI * 2) / 7); //Encoders pulse 7 times per rev
    armEncoder.reset();

    //Manipulator pivot
    m_pivot = new Spark(6);
    pivotLimitUp =  new DigitalInput(3); //channel 1 placeholder
    pivotLimitDown = new DigitalInput(4); //channel 2 placeholder
    pivotedUp = true;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //Camera stuff is in robotInit
  }

  //This function is called periodically during autonomous.   
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        //Same stuff as in driver control
        //Camera stuff in robotInit()
        driveTrain.tankDrive(-controller.getY(Hand.kLeft), -controller.getY(Hand.kRight));
        runIntake();
        controlArms();
        pivotManipulator();
        break;
    }
  }

  //This function is called periodically during operator control.
  @Override
  public void teleopPeriodic() {
    //Tank drive
    driveTrain.tankDrive(-controller.getY(Hand.kLeft) / 2, -controller.getY(Hand.kRight) / 2);
    runIntake();
    controlArms();
    pivotManipulator();
  }
  
  //Called periodically during test mode
  @Override
  public void testPeriodic() {
    driveTrain.tankDrive(-controller.getY(Hand.kLeft), -controller.getY(Hand.kRight));
  }

  //Ball intake
  public void runIntake(){
    double triggerValue;
    //Rollers run in
    if(controller.getTriggerAxis(Hand.kLeft) > 0 && controller.getTriggerAxis(Hand.kRight) == 0){
      triggerValue = controller.getTriggerAxis(Hand.kLeft);
      m_intakeLeft.setSpeed(-triggerValue);
      m_intakeRight.setSpeed(triggerValue);
    }
    //Rollers run out
    if(controller.getTriggerAxis(Hand.kRight) > 0 && controller.getTriggerAxis(Hand.kLeft) == 0){
      triggerValue = controller.getTriggerAxis(Hand.kRight);
      m_intakeLeft.setSpeed(triggerValue);
      m_intakeRight.setSpeed(-triggerValue);
    }
  }

  //Arm control
  public void controlArms(){
    //If left bumper pressed, make the arm go up
    if (controller.getBumper(Hand.kLeft)){
      m_armLeft.setSpeed(-1.0);
      m_armRight.setSpeed(1.0);
    }
    //Else if right bumper pressed, make the arm go down
    else if(controller.getBumper(Hand.kRight)){
      m_armLeft.setSpeed(1.0);
      m_armRight.setSpeed(-1.0);
    }
    //And if neither is pressed, stop the motors
    else{
      m_armLeft.setSpeed(0.0);
      m_armRight.setSpeed(0.0);
    }
  }

  //Without limit switches
  public void pivotManipulator(){
    if(controller.getBButton()){ //If up, pivot down
      m_pivot.setSpeed(1.0);
    }
    else if(controller.getAButton()){ //If down, pivot up
      m_pivot.setSpeed(-1.0);
    }
    else{
      m_pivot.setSpeed(0.0);
    }
  }
}
/* //With limit switches
  public void pivotManipulator(){
    //If we pressed the A button and we're at the bottom
    if(controller.getAButton() && !pivotedUp){
      m_pivot.setSpeed(-1.0);
      pivotedUp = !pivotedUp;
      //If we've pressed the upper limit switch, stop the motor
      if(pivotLimitUp.get()){
        m_pivot.setSpeed(0.0);
        pivotedUp = true;
      }
    }
    //Else if we've pressed the button and we're at the top
    else if(controller.getAButton() && pivotedUp){
      m_pivot.setSpeed(1.0);
      pivotedUp = !pivotedUp;
      //If we've pressed the bottom limit switch, stop the motor
      if(pivotLimitDown.get()){
        m_pivot.setSpeed(1.0);
        pivotedUp = false;
      }
    } else{
      m_pivot.setSpeed(0.0);
    }
  }
}
*/


/* Need to finish once we start testing
 //Arm code with encoders
public void controlArms(){
  double previousAngDistance, previousHeight = 0; //this should work
  int armLength = 22; //inches
  if(controller.getBumperPressed(Hand.kLeft)){
    previousAngDistance = armEncoder.getDistance();
    previousHeight = previousAngDistance * armLength;

    m_armLeft.setSpeed(1.0);
    m_armRight.setSpeed(-1.0); 
  }

  double currentAngDistance = armEncoder.getDistance();
  double currentHeight = currentAngDistance * armLength;
  /*If the motors have moved a certain distance from the previous distance
   *based on the encoder, stop the motors.
   *That difference of distance is based on how far each step will be
   * and is the height of the arm. 
  if((currentHeight - previousHeight == HEIGHT_STEP) || (currentHeight - previousHeight == ROCKET_CARGO_STEP)){ //might change to >=
     m_armLeft.setSpeed(0.0);
     m_armRight.setSpeed(0.0);
  }
 
} */
