//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer.ControlMode;
import frc.robot.subsystems.SUBDrive;
import frc.robot.RobotContainer;
import frc.utils.RoaringUtils;

public class CMDDrive extends Command {
  /** Creates a new driveRobot. */
  double angle = 0;

  double deadzone = 0.3; // variable for amount of deadzone
  double y = 0;//  variable for forward/backward movement
  double x = 0;//  variable for side to side movement
  double turn = 0;//;  variable for turning movement
  PIDController turnController;
  double pastTurn = 0;

  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system.  Note that the      */
  /* SmartDashboard in Test mode has support for helping you tune    */
  /* controllers by displaying a form where you can enter new P, I,  */
  /* and D constants and test the mechanism.                         */

  private static final double kP = 0.015;
  private static final double kI = 0.00;
  private static final double kD = 0.00;
  private static final double kToleranceDegrees = 5.0f;
  private boolean rotateToAngle;

  private CommandXboxController OIDriver1Controller = RobotContainer.getDriverController1();
  private final SUBDrive kSubDrive; 

  public CMDDrive(SUBDrive robotDrive) {
    //Use addRequirements() here to declare subsystem dependencies.
    this.kSubDrive = robotDrive;
    addRequirements(robotDrive);
  }

  //Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(kP, kI, kD);
    turnController.enableContinuousInput(-180.0f,  180.0f);
    turnController.setTolerance(kToleranceDegrees);
    //angle = kSubDrive.getHeading();
    //turnController.setSetpoint(kSubDrive.getHeading());
  }

  /**
    * Executes the robot's driving behavior.
    */
  @Override
  public void execute() {
    //Initialize variables
    x = 0;
    y = 0;
    turn = 0;

    //Get joystick input values and apply deadband
    if (RobotContainer.getControlMode() == ControlMode.Drone) {
      y = RoaringUtils.DeadzoneUtils.LinearDeadband(OIDriver1Controller.getRightY(), 0.1);
      x = RoaringUtils.DeadzoneUtils.LinearDeadband(OIDriver1Controller.getRightX(), 0.1);
      turn = RoaringUtils.DeadzoneUtils.LinearDeadband(OIDriver1Controller.getLeftX(), 0.1);
    } else {
      y = RoaringUtils.DeadzoneUtils.LinearDeadband(OIDriver1Controller.getLeftY(), 0.1);
      x = RoaringUtils.DeadzoneUtils.LinearDeadband(OIDriver1Controller.getLeftX(), 0.1);
      turn = RoaringUtils.DeadzoneUtils.LinearDeadband(OIDriver1Controller.getRightX(), 0.1);
    }

    // Determine if robot is at setpoint and needs to rotate to angle
    boolean isAtSetpoint = turnController.atSetpoint();
    if (isAtSetpoint && rotateToAngle) {
      rotateToAngle = false;
    } else if (!isAtSetpoint && rotateToAngle) {
      rotateToAngle = true;
    } else {
      rotateToAngle = false;
    }

    //Adjust setpoint if turn value changes
    if (turn != pastTurn) {
      turnController.setSetpoint(kSubDrive.getHeading());
    }

    //Adjust setpoint if "B" button is pressed
    if (OIDriver1Controller.b().getAsBoolean()) {
      turnController.setSetpoint(kSubDrive.getHeading());
    }

    //Adjust setpoint based on POV input
    if (OIDriver1Controller.povUp().getAsBoolean()) {
      turnController.setSetpoint(0.0f);
      rotateToAngle = true;
    } else if (OIDriver1Controller.povRight().getAsBoolean()) {
      turnController.setSetpoint(-90.0f);
      rotateToAngle = true;
    } else if (OIDriver1Controller.povDown().getAsBoolean()) {
      turnController.setSetpoint(179.9f);
      rotateToAngle = true;
    } else if (OIDriver1Controller.povLeft().getAsBoolean()) {
      turnController.setSetpoint(90.0f);
      rotateToAngle = true;
    }

    // Calculate rotation rate based on setpoint and joystick input
    double currentRotationRate;
    if (rotateToAngle) {
      currentRotationRate = MathUtil.clamp(turnController.calculate(kSubDrive.getHeading()), -1, 1);
    } else {
      currentRotationRate = -turn;
    }

    //Calculate rotation rate if turn value is 0
    if (turn == 0) {
      currentRotationRate = MathUtil.clamp(turnController.calculate(kSubDrive.getHeading()), -1, 1);
    }
 
    //currentRotationRate = turn;
    //Drive the robot based on joystick input and rotation rate
    if (y == 0 && x == 0 && turn == 0) { //&& isAtSetpoint) {
      kSubDrive.setX();
    } else {
      kSubDrive.drive(
        -y *0.8,
        -x * 0.8,
        currentRotationRate *0.8,
        RobotContainer.isFieldOriented(),
        RobotContainer.isRateLimited()
      );
    }

    //Update past turn value
    pastTurn = turn;

    //Reset gyro, adjust setpoint, and set rumble if "Y" button is pressed
    if (OIDriver1Controller.y().getAsBoolean()) {
      kSubDrive.resetGyro();
      turnController.setSetpoint(kSubDrive.getHeading());
      OIDriver1Controller.getHID().setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      OIDriver1Controller.getHID().setRumble(RumbleType.kBothRumble,0);
    }
  }

  //Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //nothing to end
  }

  //Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}

