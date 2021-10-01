/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import com.ctre.phoenix.motorcontrol.can.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Encoder;
/*import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;*/
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import frc.robot.Constants;

import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final Servo beakActuator = new Servo(0); 
  private final double INTAKE_SPEED = -1;
  private final double OUTTAKE_SPEED = .5;
  private final double TOPOUTTAKE_SPEED = 1;
  private final double TOPINTAKE_SPEED = 0.5;
  private final double Flaco_SPEED = -1 ;
 // private final double
  double desiredDistance = 240;
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  private static final int kEncoderPortA = 0;
  private static final int kEncoderPortB = 1;
  private Encoder m_encoder;
  private static final int kEncoderPortC = 2;
  private static final int kEncoderPortD = 3;
  private Encoder m_encoder2;
  double kP = .001;
  private Command m_autonomousCommand;
  // private final DifferentialDrive m_robotDrive = new DifferentialDrive(new WPI_VictorSPX(3), new WPI_VictorSPX(4));
  // drive motors
    private final WPI_VictorSPX m_leftMotor = new WPI_VictorSPX(5);
    private final WPI_VictorSPX m_rightMotor = new WPI_VictorSPX(3);
    private final WPI_VictorSPX m_leftfollow = new WPI_VictorSPX(2);
    private final WPI_VictorSPX m_rightfollow = new WPI_VictorSPX(4);
  
    private final WPI_TalonSRX m_TopIntakeMotor1 = new WPI_TalonSRX(6); // gray
    private final WPI_TalonSRX m_BottomIntakeMotor2 = new WPI_TalonSRX(8); // pink :)
    private final WPI_TalonSRX m_TopIntakeMotor2 = new WPI_TalonSRX(7); // green 
    private final WPI_TalonSRX m_Pwnf = new WPI_TalonSRX(9); // white
  
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    private final XboxController m_stick = new XboxController(0);
    private final Joystick m_Extreme1 = new Joystick(1);
    private final Joystick  m_Extreme2 = new Joystick(2);
    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    //private final I2C.Port i2cPort = I2C.Port.kOnboard;
    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
//    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  
    // public static class ColorSensorV3.RawColor
   // private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Timer m_timer = new Timer();
    private final double athenatime = m_timer.get();

    private RobotContainer m_robotContainer;

    /**
    * Note: Any example colors should be calibrated as the user needs, these
    * are here as a basic example.
    
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);*/
    private SendableChooser<Command> chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */

  @Override
  public void robotInit() {
    //chooser.addDefault("Right", new RobotDrive());
    chooser.setDefaultOption("Right", m_autonomousCommand);
   /* m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(true);
    m_leftfollow.setInverted(true);
    m_rightfollow.setInverted(true);*/
    /*chooser.addOption("Left", new AutoLeft());
    
    
    chooser.addObject("Left OOW", new AutoLeftOutOfWay());
    chooser.addObject("Center", new AutoCenter());
    chooser.addObject("Cross the Line", new AutoCrossTheLine());*/
  
    SmartDashboard.putData("Auto mode", chooser);

    CameraServer.getInstance().startAutomaticCapture();
     //Get the default instance of NetworkTables that was created automatically
       //when your program starts
       NetworkTableInstance inst = NetworkTableInstance.getDefault();
        //Get the table within that instance that contains the data. There can
       //be as many tables as you like and exist to make it easier to organize
       //your data. In this case, it's a table called datatable.
       NetworkTable table = inst.getTable("datatable");
// swag nat and caro were here feb 20 2021
       //Get the entries within that table that correspond to the X and Y values
       //for some operation in your program.
       xEntry = table.getEntry("X");
       yEntry = table.getEntry("Y");
    m_leftfollow.follow(m_leftMotor);
    m_rightfollow.follow(m_rightMotor);
    
    
      
    
    
      
    m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    m_encoder2 = new Encoder(kEncoderPortC, kEncoderPortD);

    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.

    m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    m_encoder2.setDistancePerPulse((Math.PI * 6) / 360.0);  
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_encoder.reset();
/*
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    
*/

 
  }
  double x = 0;
  double y = 0;

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
    SmartDashboard.putNumber("Encoder2", m_encoder2.getDistance());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

        /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     
    String colorString;
    final Color detectedColor = m_colorSensor.getColor();
    final RawColor detectedRawColor = m_colorSensor.getRawColor();
    final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);*/

    /**
     * The sensor returns a raw IR value of the infrared light detected.
     
    final double IR = m_colorSensor.getIR();*/

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    //Color detectedrawcolor = GetRawColor(detectedColor);
    //SmartDashboard.putNumber("Leslie", detectedColor.getRawColor());
 
    boolean booleanRed = false;
    boolean booleanGreen = false;
    boolean booleanBlue = false;
    boolean booleanYellow = false;

    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    //final int proximity = m_colorSensor.getProximity();

   // SmartDashboard.putNumber("Proximity", proximity);
    String egg;
    egg = "";
    String gameData;
gameData = DriverStation.getInstance().getGameSpecificMessage();
if(gameData.length() > 0)
{
  switch (gameData.charAt(0))
  {
    case 'B' :
      //Blue case code
      SmartDashboard.putString("FMS Color", "Blue");
      egg="Blue";
      break;
    case 'G' :
      //Green case code
      SmartDashboard.putString("FMS Color", "Green");
      egg="Green";
      break;
    case 'R' :
      //Red case code 
      SmartDashboard.putString("FMS Color", "Red");
      egg="Red";
      break;
    case 'Y' :
      //Yellow case code
      SmartDashboard.putString("FMS Color", "Yellow");
      egg="Yellow";
      break;
    default :
      //This is corrupt data
      break;
  }
} else {
  //Code for no data received yet
}
/*if (m_stick.getXButtonPressed()) {
  // drive forwards half speed
  if (match.color == kBlueTarget){
    // m_robotDrive.arcadeDrive(0.5, 0.0); 
     colorString = "Blue";
     booleanBlue =true;
     if (egg != "Blue"){
  //    m_robotDrive.arcadeDrive(0.5, 0.0); 
     }else {
  //    m_robotDrive.stopMotor();
     }
   }else if (match.color == kRedTarget){
     colorString = "Red";
    // m_robotDrive.stopMotor();
     booleanRed = true;
     if (egg != "Red"){
    //  m_robotDrive.arcadeDrive(0.5, 0.0); 
     }else {
    //  m_robotDrive.stopMotor();
     }
   }else if (match.color == kGreenTarget){
     //m_robotDrive.arcadeDrive(0.5, 0.0); 
     colorString = "Green";
     booleanGreen = true;   
     if (egg != "Green"){
   //   m_robotDrive.arcadeDrive(0.5, 0.0); 
     }else {
   //   m_robotDrive.stopMotor();
     }
    }
   else if (match.color == kYellowTarget){
    // m_robotDrive.stopMotor();
     colorString = "Yellow";
     booleanYellow = true;   
     if (egg != "Yellow"){
   //   m_robotDrive.arcadeDrive(0.5, 0.0); 
     }else {
     // m_robotDrive.stopMotor();
     }
    }
   else {
     colorString = "unknown";
   }
} else {
  //m_robotDrive.stopMotor(); // stop robot
}*/
 SmartDashboard.putBoolean("isRed", booleanRed);
 SmartDashboard.putBoolean("isBlue", booleanBlue);
 SmartDashboard.putBoolean("isYellow", booleanYellow);
 SmartDashboard.putBoolean("isGreen", booleanGreen);
  }
  
  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_timer.start();
    m_encoder.reset();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
 //START AUTO
   @Override
  public void autonomousPeriodic() {

    //System.out.println("distance: "+m_encoder.getDistance());
   // Drive for 2 seconds
  // if (m_timer.get() < 2.0) {

/*
    if (m_encoder.getDistance() < desiredDistance) {
      m_robotDrive.arcadeDrive(0.5, 0.0);
      if(athenatime > 5.0){
        m_robotDrive.stopMotor(); // stop robot
        m_BottomIntakeMotor1.set(OUTTAKE_SPEED);
        m_TopIntakeMotor.set(TOPOUTTAKE_SPEED);
    }
  }
  else{
    m_robotDrive.stopMotor();
    m_BottomIntakeMotor1.set(OUTTAKE_SPEED);
    m_TopIntakeMotor.set(TOPOUTTAKE_SPEED);
  }*/
   // while (m_timer.get() <10.0){

    /*
    if(m_timer.get() < 4.0){
      m_robotDrive.arcadeDrive(-0.5, 0.0);// drive forward half speed 
    }

    else if (m_timer.get() > 4.0 && m_timer.get()  < 4.5 || m_timer.get()>5.33 && m_timer.get()< 5.66 || m_timer.get()>6.0 && m_timer.get()< 6.33){
      //Whoever thought of using the "OR" operator for the else if loops....VERY SMART COOKIE!!! - Kylel
      m_robotDrive.stopMotor();
      m_TopIntakeMotor1.set(OUTTAKE_SPEED); // drive forwards half speed
      m_BottomIntakeMotor2.set(TOPOUTTAKE_SPEED); // drive forwards half speed
      System.out.println("distance: "+m_encoder.getDistance());
    }

    else if (m_timer.get() > 5.0 && m_timer.get()  < 5.33 ||m_timer.get() > 5.66 && m_timer.get()  < 5.99 || m_timer.get() > 6.33 && m_timer.get()  < 6.66 ){
      m_robotDrive.stopMotor();
      m_TopIntakeMotor1.set(0);
      m_BottomIntakeMotor2.set(0);
    }*/
   /* else if (m_timer.get()>6.0 && m_timer.get()< 7.0) {
      m_robotDrive.stopMotor();
      m_BottomIntakeMotor1.set(OUTTAKE_SPEED); // drive forwards half speed
      m_TopIntakeMotor.set(TOPOUTTAKE_SPEED); // drive forwards half speed
    }

    else{
      m_robotDrive.stopMotor();
      m_TopIntakeMotor1.set(0);
      m_BottomIntakeMotor2.set(0); 
    }
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    }
*/

  //}


    
    //else if (m_encoder.getDistance() == desiredDistance || m_timer.get() > 2.0) {
  } //END AUTO

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // Reset the encoders
    m_encoder.reset();
    m_encoder2.reset();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   /* m_encoder.setDistancePerPulse(1./256.);
    m_encoder2.setDistancePerPulse(1./256.);*/

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
     //Using the entry objects, set the value to a double that is constantly
       //increasing. The keys are actually "/datatable/X" and "/datatable/Y".
       //If they don't already exist, the key/value pair is added.
       xEntry.setDouble(x);
       yEntry.setDouble(y);
       x += 0.05;
       y += 1.0;

       if (m_Extreme1.getTrigger()) {
        m_TopIntakeMotor1.set(-TOPINTAKE_SPEED);
        m_BottomIntakeMotor2.set(TOPINTAKE_SPEED);
         }     else {
          m_TopIntakeMotor1.set(0);
          m_BottomIntakeMotor2.set(0);
         // stop motor
        }

        if (m_Extreme2.getTrigger()){
          m_TopIntakeMotor2.set(-TOPOUTTAKE_SPEED);
          m_Pwnf.set(-TOPOUTTAKE_SPEED);
        }     else {
          m_TopIntakeMotor2.set(0);
          m_Pwnf.set(0);
        }
        
      /*  double error = m_encoder.getDistance() + m_encoder2.getDistance();

       if (m_encoder.getDistance() < desiredDistance && m_encoder2.getDistance() < desiredDistance) {
//tank drive takes in inputs from the left & right
        m_robotDrive.tankDrive(.625 + kP * error, .625 - kP * error);
        System.out.println("Motor things: "+(kP * error));
    }
    else{
      m_robotDrive.stopMotor();
    }*/

    m_robotDrive.arcadeDrive(m_Extreme1.getY(Hand.kLeft), m_Extreme2.getX(Hand.kRight));
    
  
  /********** THIS IS THE TWO JOYSTICK TANK DRIVE
     m_robotDrive.tankDrive(-m_Extreme1.getY(Hand.kRight), -m_Extreme2.getY(Hand.kLeft));
     ******/

    /* THIS IS THE XBOX CONTROLLER TANK DRIVE*/
    // m_robotDrive.tankDrive(-m_stick.getY(Hand.kRight), -m_stick.getY(Hand.kLeft));

     /* THIS IS THE XBOX CONTROLLER ARCADE DRIVE 
     beakActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); 

     final double move = m_Extreme1.getY(Hand.kLeft);
     final double turn = m_Extreme2.getX(Hand.kRight);
     if(m_stick.getBumper(Hand.kLeft)){ // arguments might have to be set as a bool varialbe and called
      m_robotDrive.arcadeDrive(-move/2, turn/2); // set the control to half normal speed
      beakActuator.setSpeed(-1.0);
      m_Pwnf.set(0);
      while(m_Extreme1.getRawButton(2)){
        m_Pwnf.set(Flaco_SPEED);
      }
      while(m_Extreme2.getRawButton(2)){
        m_Pwnf.set(-Flaco_SPEED);
     }
    }
     else{
     m_robotDrive.arcadeDrive(-m_Extreme1.getY(Hand.kLeft), m_Extreme2.getX(Hand.kRight));
     beakActuator.setSpeed(1.0);
    }


     if(m_stick.getAButton()){
      beakActuator.setSpeed(1.0); // to open 
     }
    if(m_stick.getBButton()){
      beakActuator.setSpeed(-1.0); // to close
    }
    if(m_stick.getXButton()){
      m_Pwnf.set(Flaco_SPEED);
    }
    if(m_stick.getYButton()){
      m_Pwnf.set(0);
    }

     if (m_Extreme1.getTrigger()) {
      m_TopIntakeMotor1.set(INTAKE_SPEED);
      m_BottomIntakeMotor2.set(TOPINTAKE_SPEED);
       } 
       
    else {
      m_TopIntakeMotor1.set(0);
      m_BottomIntakeMotor2.set(0);
     // stop motor
    }
  } */
    /******** THIS IS THE XBOXCONTROLLER THING FOR INTAKE YEA
     *     if (m_stick.getYButton()) {
      m_BottomIntakeMotor1.set(OUTTAKE_SPEED);
      m_TopIntakeMotor.set(TOPOUTTAKE_SPEED);
       } 
       else if (m_stick.getAButton()){
        m_BottomIntakeMotor1.set(INTAKE_SPEED);
        m_TopIntakeMotor.set(TOPINTAKE_SPEED);
      // stop motor
    }else {
      m_BottomIntakeMotor1.set(0);
      m_TopIntakeMotor.set(0);
     // stop motor
   }*/
    /*
        if (m_stick.getYButton()) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }*/
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void DoNothing(){
    m_autonomousCommand.cancel();
  }
}
