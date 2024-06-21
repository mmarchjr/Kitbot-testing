// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SUBMAXSwerveModule {
  private final CANSparkMax kDrivingSparkMax;
  private final CANSparkMax kTurningSparkMax;

  private final RelativeEncoder kDrivingEncoder;
  private final AbsoluteEncoder kTurningEncoder;

  private final SparkPIDController kDrivingPIDController;
  private final SparkPIDController kTurningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SUBMAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    kDrivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    kTurningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    kDrivingSparkMax.restoreFactoryDefaults();
    kTurningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    kDrivingEncoder = kDrivingSparkMax.getEncoder();
    kTurningEncoder = kTurningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    kDrivingPIDController = kDrivingSparkMax.getPIDController();
    kTurningPIDController = kTurningSparkMax.getPIDController();
    kDrivingPIDController.setFeedbackDevice(kDrivingEncoder);
    kTurningPIDController.setFeedbackDevice(kTurningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    kDrivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    kDrivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    kTurningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    kTurningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    kTurningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    kTurningPIDController.setPositionPIDWrappingEnabled(true);
    kTurningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    kTurningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    kDrivingPIDController.setP(ModuleConstants.kDrivingP);
    kDrivingPIDController.setI(ModuleConstants.kDrivingI);
    kDrivingPIDController.setD(ModuleConstants.kDrivingD);
    kDrivingPIDController.setFF(ModuleConstants.kDrivingFF);
    kDrivingPIDController.setOutputRange(
      ModuleConstants.kDrivingMinOutput,
      ModuleConstants.kDrivingMaxOutput
    );

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    kTurningPIDController.setP(ModuleConstants.kTurningP);
    kTurningPIDController.setI(ModuleConstants.kTurningI);
    kTurningPIDController.setD(ModuleConstants.kTurningD);
    kTurningPIDController.setFF(ModuleConstants.kTurningFF);
    kTurningPIDController.setOutputRange(
      ModuleConstants.kTurningMinOutput,
      ModuleConstants.kTurningMaxOutput
    );

    kDrivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    kTurningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    kDrivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    kTurningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    kDrivingSparkMax.burnFlash();
    kTurningSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(kTurningEncoder.getPosition());
    kDrivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(kDrivingEncoder.getVelocity(),
      new Rotation2d(kTurningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
      kDrivingEncoder.getPosition(),
      new Rotation2d(kTurningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
      new Rotation2d(kTurningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    kDrivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    kTurningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    kDrivingEncoder.setPosition(0);
  }
}
