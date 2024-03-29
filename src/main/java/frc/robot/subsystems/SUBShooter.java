// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.kFeedCurrentLimit;
import static frc.robot.Constants.LauncherConstants.kFeederID;
import static frc.robot.Constants.LauncherConstants.kIntakeFeederSpeed;
import static frc.robot.Constants.LauncherConstants.kIntakeLauncherSpeed;
import static frc.robot.Constants.LauncherConstants.kLauncherCurrentLimit;
import static frc.robot.Constants.LauncherConstants.kLauncherID1;
import static frc.robot.Constants.LauncherConstants.kLauncherID2;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class SUBShooter extends SubsystemBase {

  private CANSparkMax launchWheel1;
  private CANSparkMax launchWheel2;

  private CANSparkMax feedWheel;
  private PIDController pid = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);
  private RelativeEncoder encoder;
  private Double velocity;

  /** Creates a new Launcher. */
  public SUBShooter() {
    launchWheel1 = new CANSparkMax(kLauncherID1, MotorType.kBrushless);
    launchWheel2 = new CANSparkMax(kLauncherID2, MotorType.kBrushless);
    feedWheel = new CANSparkMax(kFeederID, MotorType.kBrushless);

    pid.setP(LauncherConstants.kP);
    pid.setI(LauncherConstants.kI);
    pid.setD(LauncherConstants.kD);
    
    encoder = launchWheel1.getEncoder();
  }

  public void init () {
    launchWheel1.setSmartCurrentLimit(kLauncherCurrentLimit);
    feedWheel.setSmartCurrentLimit(kFeedCurrentLimit);
    feedWheel.setIdleMode(IdleMode.kBrake);
    launchWheel1.setIdleMode(IdleMode.kCoast);
    launchWheel2.setIdleMode(IdleMode.kCoast);
    encoder = launchWheel1.getEncoder();
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
    launchWheel1.set(speed);
    launchWheel2.set(speed);
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
  
  public double getRPM() {
    return encoder.getVelocity();
  }
}