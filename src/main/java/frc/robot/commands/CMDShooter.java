// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUBShooter;

public class CMDShooter extends Command {
  private SUBShooter subShooter;
  private static CommandXboxController xbox = RobotContainer.getDriverController2();
  /** Creates a new CMDShooter. */
  public CMDShooter(SUBShooter sub) {
    // addRequirements(RobotContainer.m_SUBShooter);
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(sub);
    this.subShooter = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedvalue = 0;
    double launchvalue= 0;
    if (xbox.leftTrigger().getAsBoolean()) {feedvalue=0.5;}
    if (xbox.leftBumper().getAsBoolean()) {feedvalue=-0.5;}
    // if (xbox.rightTrigger().getAsBoolean()) {launchvalue=0.25;}
    // if (xbox.rightBumper().getAsBoolean()) {launchvalue=-0.25;}

    subShooter.setLaunchWheel(launchvalue);
    subShooter.setFeedWheel(feedvalue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
 }

  //  if (robotChooser.getSelected() == RobotMode.KitBot) {
    //  OIDriverController2
    //  .rightBumper()
    //  .whileTrue(
    //    new PrepareLaunch(kSUBShooter)
    //    .withTimeout(LauncherConstants.kLauncherDelay)
    //    .andThen(new LaunchNote(kSUBShooter))
    //    .handleInterrupt(() -> kSUBShooter.stop())
    //  );
    //  OIDriverController2.leftBumper().whileTrue(kSUBShooter.getIntakeCommand());

  //  }
