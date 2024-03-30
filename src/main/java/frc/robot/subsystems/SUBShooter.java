// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.kFeedCurrentLimit;
import static frc.robot.Constants.LauncherConstants.kFeederID;
import static frc.robot.Constants.LauncherConstants.kLauncherCurrentLimit;
import static frc.robot.Constants.LauncherConstants.kLauncherID1;
import static frc.robot.Constants.LauncherConstants.kLauncherID2;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.utils.RoaringUtils.ToleranceChecker;

public class SUBShooter extends SubsystemBase {

  private CANSparkMax launchWheel1;
  private CANSparkMax launchWheel2;

  private CANSparkMax feedWheel;
  //private PIDController pid = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);
  private SparkPIDController upperPID;
  private SparkPIDController lowerPID;
  private RelativeEncoder upperEncoder;
  private RelativeEncoder lowerEncoder;
  double velocity = 0;

  /** Creates a new Launcher. */
  public SUBShooter() {
    launchWheel1 = new CANSparkMax(kLauncherID1, MotorType.kBrushless);
    launchWheel2 = new CANSparkMax(kLauncherID2, MotorType.kBrushless);
    feedWheel = new CANSparkMax(kFeederID, MotorType.kBrushless);
    upperPID = launchWheel1.getPIDController();
    lowerPID = launchWheel2.getPIDController();
    launchWheel1.setSmartCurrentLimit(kLauncherCurrentLimit);
    feedWheel.setSmartCurrentLimit(kFeedCurrentLimit);
    feedWheel.setIdleMode(IdleMode.kBrake);
    launchWheel1.setIdleMode(IdleMode.kCoast);
    launchWheel2.setIdleMode(IdleMode.kCoast);
    upperEncoder = launchWheel1.getEncoder();
    lowerEncoder = launchWheel2.getEncoder();
    upperPID.setFeedbackDevice(upperEncoder);
    lowerPID.setFeedbackDevice(lowerEncoder);
    upperPID.setOutputRange(-1,1);

    lowerPID.setOutputRange(-1,1);

    upperPID.setP(LauncherConstants.kP);
    upperPID.setI(LauncherConstants.kI);
    upperPID.setD(LauncherConstants.kD);
    upperPID.setFF(LauncherConstants.kFTop);

    lowerPID.setP(LauncherConstants.kP);
    lowerPID.setI(LauncherConstants.kI);
    lowerPID.setD(LauncherConstants.kD);
    lowerPID.setFF(LauncherConstants.kFbottom);

    upperPID.setPositionPIDWrappingMaxInput(1);
    upperPID.setPositionPIDWrappingMinInput(-1);
    lowerPID.setPositionPIDWrappingMaxInput(1);
    lowerPID.setPositionPIDWrappingMinInput(-1);

    upperPID.setPositionPIDWrappingEnabled(true);
    lowerPID.setPositionPIDWrappingEnabled(true);    

    //pid.setP(LauncherConstants.kP);
    //pid.setI(LauncherConstants.kI);
    //pid.setD(LauncherConstants.kD);
  }

  public void init () {

  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
      // When the command is initialized, set the wheels to the intake speed values
      () -> {
        setFeedWheel(0.7);
        setLaunchWheel(0);
      },
      // When the command stops, stop the wheels
      () -> {
        stop();
      }
    );
  }

public Command getIdleCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
      // When the command is initialized, set the wheels to the intake speed values
      () -> {
        setFeedWheel(0);
        setLaunchWheel(1);
      },
      // When the command stops, stop the wheels
      () -> {
        stop();
      }
    );
  }

  public Command getLaunchCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
      // When the command is initialized, set the wheels to the intake speed values
      () -> {
        setFeedWheel(1);
        setLaunchWheel(1);
      },
      // When the command stops, stop the wheels
      () -> {
        stop();
      }
    );
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    //launchWheel1.set(speed);
    //launchWheel2.set(speed);
    velocity = speed * LauncherConstants.kShooterRPM;
    upperPID.setReference(velocity,ControlType.kVelocity);
    lowerPID.setReference(velocity,ControlType.kVelocity);

  }
         
  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    feedWheel.set(speed);
  }

  public void setWheels(Double feed, Double Launch) {
    setFeedWheel(feed);
    setLaunchWheel(Launch);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    launchWheel1.set(0);
    feedWheel.set(0);
  }
  
  public double getUpperRPM() {
    return upperEncoder.getVelocity();
  }
    public double getLowerRPM() {
    return lowerEncoder.getVelocity();
  }
  public boolean upperSetpointReached() {
    return ToleranceChecker.isWithinTolerance(upperEncoder.getVelocity(),velocity , LauncherConstants.kTolerance);
  }
  public boolean lowerSetpointReached() {
    return ToleranceChecker.isWithinTolerance(lowerEncoder.getVelocity(),velocity , LauncherConstants.kTolerance);
  }
  public boolean bothSetpointsReached() {
    return (upperSetpointReached() && lowerSetpointReached());
  }
}