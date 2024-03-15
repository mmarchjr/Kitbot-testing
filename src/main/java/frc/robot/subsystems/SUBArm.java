// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ModuleConstants;
import frc.utils.UniMotor;
import frc.utils.UniMotor.UniMotorType;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;



public class SUBArm extends SubsystemBase {
  
  /** Creates a new SUBArm. */
   public UniMotor armMotor1 = new UniMotor(Constants.ArmConstants.kArmMotor1, UniMotorType.SparkMAX);
   UniMotor armMotor2 = new UniMotor(Constants.ArmConstants.kArmMotor2, UniMotorType.SparkMAX);
   AbsoluteEncoder encoder = armMotor1.sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
   double setpoint = ArmConstants.kIntakePosition;
   SparkPIDController m_turningPIDController;

   PIDController pid = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);

  public SUBArm(
  ) {

    pid.setTolerance(Units.degreesToRotations(2));
    armMotor1.sparkMax.restoreFactoryDefaults();
    armMotor1.setSmartCurrentLimit(ArmConstants.kMotorCurrentLimit);
    armMotor2.setSmartCurrentLimit(ArmConstants.kMotorCurrentLimit);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningPIDController = armMotor1.sparkMax.getPIDController();
    m_turningPIDController.setFeedbackDevice(encoder);
    m_turningPIDController.setOutputRange(-1,1);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.


    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    encoder.setInverted(true);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(0);
    m_turningPIDController.setPositionPIDWrappingMaxInput(1);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!


    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ArmConstants.kP);
    m_turningPIDController.setI(ArmConstants.kI);
    m_turningPIDController.setD(ArmConstants.kD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);

    armMotor1.sparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    armMotor1.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    armMotor1.sparkMax.burnFlash();


armMotor2.follow(armMotor1);
armMotor1.sparkMax.setIdleMode(IdleMode.kBrake);
armMotor2.sparkMax.setIdleMode(IdleMode.kBrake);
m_turningPIDController.setReference(encoder.getPosition(), CANSparkMax.ControlType.kPosition);


  }

  public void setPosition(double position){
    setpoint = position; 
    //double motorPower = pid.calculate(encoder.getPosition(), setpoint);
    m_turningPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    //armMotor1.set(-position);
  }
   public void changePosition(double position){
    //setpoint = position; 
    //double motorPower = pid.calculate(encoder.getPosition(), setpoint);
    //m_turningPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    armMotor1.set(position);
  }
    public double getPosition(){
      return setpoint;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("armP", Constants.ArmConstants.kP);
    //SmartDashboard.putNumber("armI", Constants.ArmConstants.kI);
    //SmartDashboard.putNumber("armD", Constants.ArmConstants.kD);
    //SmartDashboard.putNumber("pos", encoder.getPosition());
    m_turningPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);

  }
  public boolean isAtSetpoint() {
    return pid.atSetpoint();
  }
}