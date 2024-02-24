// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.Constants.LauncherConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotMode;
import frc.utils.UniMotor;
import frc.utils.UniMotor.UniMotorMode;
import frc.utils.UniMotor.UniMotorType;

public class SUBShooter extends SubsystemBase {

  UniMotor m_launchWheel1;
  UniMotor m_launchWheel2;

  UniMotor m_feedWheel;

  /** Creates a new Launcher. */
  public SUBShooter() {


  }
  public void init () {
        if (RobotContainer.robotChooser.getSelected() == RobotMode.CompBot) {
        m_launchWheel1 = new UniMotor(kLauncherID1,UniMotorType.SparkMAX);
        m_launchWheel2 = new UniMotor(kLauncherID2,UniMotorType.SparkMAX);
        m_feedWheel = new UniMotor(kFeederID,UniMotorType.SparkMAX);

    } else if (RobotContainer.robotChooser.getSelected() == RobotMode.KitBot) {
      m_launchWheel1 = new UniMotor(kLauncherID1, UniMotorType.TalonSRX);
      m_launchWheel2 = new UniMotor(kLauncherID2,UniMotorType.TalonSRX);
      m_feedWheel = new UniMotor(kFeederID,UniMotorType.TalonSRX);
    }

    //m_launchWheel2.follow(m_launchWheel1);
    m_launchWheel2.setInverted(false);
    m_launchWheel1.setSmartCurrentLimit(kLauncherCurrentLimit);
    m_launchWheel2.setSmartCurrentLimit(kLauncherCurrentLimit);
    m_feedWheel.setSmartCurrentLimit(kFeedCurrentLimit);
    m_feedWheel.setIdleMode(UniMotorMode.Brake);
    m_launchWheel1.setIdleMode(UniMotorMode.Coast);
    m_launchWheel2.setIdleMode(UniMotorMode.Coast);
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
          setFeedWheel(kIntakeFeederSpeed);
          setLaunchWheel(kIntakeLauncherSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

public Command getIdleCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(kIntakeFeederSpeed);
          setLaunchWheel(kIntakeLauncherSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  public Command getLaunchCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(0);
          setLaunchWheel(0);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel1.set(speed);
    m_launchWheel2.set(speed);
    //m_launchWheel1.setVelocitySpark(2000);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheel1.set(0);
    m_feedWheel.set(0);
  }
}