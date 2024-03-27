// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CMDArm;
import frc.robot.commands.CMDClimb;
import frc.robot.commands.CMDDrive;
import frc.robot.commands.CMDShooter;
import frc.robot.commands.CMDlights;
import frc.robot.subsystems.SUBArm;
import frc.robot.subsystems.SUBClimb;
import frc.robot.subsystems.SUBDrive;
import frc.robot.subsystems.SUBPoseEstimator;
import frc.robot.subsystems.SUBShooter;
import frc.robot.subsystems.SUBVision;
import frc.robot.subsystems.SUBlights;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final int kShooterRPM = LauncherConstants.kShooterRPM;
  public static final SUBlights m_lights = new SUBlights();
    public static final CMDlights m_lightCommand = new CMDlights(m_lights);
    public static final SUBVision kSUBVision = new SUBVision();

  //The robot's subsystems
  private static final SUBDrive kRobotDrive = new SUBDrive();
  private static final CMDDrive kDriveRobotCommand = new CMDDrive(kRobotDrive);
  public static final SUBShooter kSUBShooter = new SUBShooter();
  private static final CMDShooter kCMDShooter = new CMDShooter(kSUBShooter,kSUBVision);
  private static final SUBArm kSUBArm = new SUBArm();
  private static final CMDArm kCMDArm = new CMDArm(kSUBArm);
  private static final SUBClimb kSUBClimb = new SUBClimb();
  private static final CMDClimb kCMDClimb = new CMDClimb(kSUBClimb);

  private static final PathConstraints kPathConstraints = new PathConstraints(5, 3, 360, 15);
  private final SUBPoseEstimator kPoseEstimator = new SUBPoseEstimator( kRobotDrive,kSUBVision);
  public enum RobotMode {
    KitBot, CompBot
  }
  public enum ControlMode {
    Drone, Game
  }
  private static SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<Boolean>();
  private static SendableChooser<ControlMode> controlChooser = new SendableChooser<ControlMode>();
  private static SendableChooser<Boolean> rateLimitChooser = new SendableChooser<Boolean>();
  private static SendableChooser<RobotMode> robotChooser = new SendableChooser<RobotMode>();
  private static SendableChooser<Command> autoChooser;

  //The driver's controller
  private static CommandXboxController OIDriverController1 = new CommandXboxController(OIConstants.kDriverControllerPort);
  private static CommandXboxController OIDriverController2 = new CommandXboxController(OIConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // kRobotDrive.resetOdometry(new Pose2d(8.25,4.1, Rotation2d.fromDegrees(0)));
    //kRobotDrive.resetOdometry(PathPlannerPath.fromPathFile("2 note auto").getPreviewStartingHolonomicPose());
    

    AutoBuilder.configureHolonomic(
      kPoseEstimator::getCurrentPose,  //Robot pose supplier
      kPoseEstimator::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
      kRobotDrive::getspeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      kRobotDrive::driveRobotRelative,//  Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(10, 0.0, 0.0), // Translation PID constants
        new PIDConstants(10, 0.0, 0.0),//  Rotation PID constants
        2,//  Max module speed, in m/s
        Units.inchesToMeters(18.2), // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      kRobotDrive // Reference to this subsystem to set requirements
    );

    // Configure the button bindings

    NamedCommands.registerCommand("Take Note", kSUBShooter.getIntakeCommand().repeatedly());
    NamedCommands.registerCommand("Amp Note", new RunCommand(()->kSUBShooter.setWheels(0.5,0.1), kSUBShooter).repeatedly().withTimeout(1));
    NamedCommands.registerCommand("Arm Intake", new RunCommand(()->kSUBArm.setPosition(ArmConstants.kIntakePosition), kSUBArm).repeatedly().withTimeout(1));//.until(()->kSUBArm.isAtSetpoint()));
    NamedCommands.registerCommand("Arm Amp", new RunCommand(()->kSUBArm.setPosition(ArmConstants.kAmpPosition), kSUBArm).repeatedly().withTimeout(2));//.until(()->kSUBArm.isAtSetpoint()));
    NamedCommands.registerCommand("Arm Speaker", new RunCommand(()->kSUBArm.setPosition(ArmConstants.kSpeakerPosition), kSUBArm).repeatedly().withTimeout(1));
    NamedCommands.registerCommand("Speaker Note", new RunCommand(
      ()->kSUBShooter.setLaunchWheel(1), kSUBShooter).repeatedly().until(()->(kSUBShooter.getRPM()>kShooterRPM))
      .andThen(new RunCommand(()->kSUBShooter.setWheels(0.6,1.0),kSUBShooter).repeatedly().withTimeout(1)));
    fieldOrientedChooser.setDefaultOption("Field Oriented", true);
    fieldOrientedChooser.addOption("Robot Oriented", false);
    NamedCommands.registerCommand("Arm Up", new RunCommand(()->kSUBArm.setPosition(ArmConstants.kHoldPosition), kSUBArm).withTimeout(1));//.until(()->kSUBArm.isAtSetpoint()));
    rateLimitChooser.setDefaultOption("True", true);
    rateLimitChooser.addOption("False", false);
    
    controlChooser.setDefaultOption("Drone", ControlMode.Drone);
    controlChooser.addOption("Game", ControlMode.Game);
    robotChooser.setDefaultOption("Main Comp", RobotMode.CompBot);
    robotChooser.addOption("KitBot", RobotMode.KitBot);

   

    SmartDashboard.putData("Rate limit",rateLimitChooser);
    SmartDashboard.putData("Field oriented",fieldOrientedChooser);
    SmartDashboard.putData("Controls", controlChooser);
    //SmartDashboard.putData("Robot Select", robotChooser);

    // Configure default commands
    kSUBShooter.setDefaultCommand(kCMDShooter);
    kRobotDrive.setDefaultCommand(kDriveRobotCommand);
    kSUBArm.setDefaultCommand(kCMDArm);
    kPoseEstimator.register();
    kSUBVision.register();
    kSUBVision.periodic();
    kPoseEstimator.periodic();
    kSUBClimb.setDefaultCommand(kCMDClimb);
    m_lights.setDefaultCommand(m_lightCommand);


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("auto", autoChooser);
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    OIDriverController1.x().whileTrue(new RunCommand(
      () -> kRobotDrive.setX(),
      kRobotDrive
    ));
      
    OIDriverController2.rightBumper().whileTrue(new RunCommand(
      ()->kSUBShooter.setLaunchWheel(1), kSUBShooter).repeatedly().until(()->(kSUBShooter.getRPM() >kShooterRPM)).withTimeout(3)
      .andThen(new RunCommand(()->kSUBShooter.setWheels(0.6,1.0)).repeatedly().withTimeout(1)));
    //OIDriverController1.rightTrigger(0.1)
    //  .whileTrue(AutoBuilder.pathfindToPose(new Pose2d(1.75,5.5,Rotation2d.fromDegrees(180)), kPathConstraints));
    OIDriverController2.y().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kAmpPosition), kSUBArm));
    OIDriverController2.a().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kIntakeUpPosition), kSUBArm).repeatedly().withTimeout(0.5).andThen(()-> kSUBArm.setPosition(ArmConstants.kIntakePosition)));
    OIDriverController2.x().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kHoldPosition), kSUBArm));
    OIDriverController2.b().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kSpeakerPosition), kSUBArm));
    OIDriverController1.a().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kInsidePosition), kSUBArm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static CommandXboxController getDriverController1() {
    return new CommandXboxController(OIConstants.kDriverControllerPort);
  }

  public static CommandXboxController getDriverController2() {
    return new CommandXboxController(OIConstants.kDriverControllerPort2);
  }

  public static RobotMode getRobotMode() {
    return robotChooser.getSelected();
  }

  public static ControlMode getControlMode() {
    return controlChooser.getSelected();
  }

  public static boolean isFieldOriented() {
    return fieldOrientedChooser.getSelected();
  }

  public static boolean isRateLimited() {
    return rateLimitChooser.getSelected();
  }

  public static SUBDrive getDriveSubsystem() {
    return kRobotDrive;
  }
}