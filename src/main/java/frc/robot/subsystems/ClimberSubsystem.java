// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimits;

public class ClimberSubsystem extends SubsystemBase {


  private SparkMax climberMotor;
  private SparkMaxConfig climberConfig;
  private AbsoluteEncoder climberEncoder;
  
  private SparkClosedLoopController climberPID;

  private final double climberUpPosition = 310; //encoder value when climber is up
  private final double climberDownPosition = 0; //encoder value when climber is down

  private final double kP = 0.1;
  private final double kI = 0;
  private final double kD = 0;

  private final double kMax = 1;
  private final double kMin = -1;

  private final double climberRatio = 25;
  
  /** Creates a new ClimberSubsystem. */

  public ClimberSubsystem() {

    climberMotor = new SparkMax(21, MotorType.kBrushless);

    climberConfig = new SparkMaxConfig();

    climberPID = climberMotor.getClosedLoopController();

    // climberEncoder = climberMotor.getAbsoluteEncoder();

    configureClimberMotor();

  }

  private void configureClimberMotor(){

    climberConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    climberConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(kP)
    .i(kI)
    .d(kD)
    .outputRange(kMin, kMax);

  climberConfig.smartCurrentLimit(CurrentLimits.Neo500)
  .inverted(true)
  .idleMode(IdleMode.kBrake)
  
  .softLimit
  .forwardSoftLimit(350)
  .reverseSoftLimit(0)
  .forwardSoftLimitEnabled(true)
  .reverseSoftLimitEnabled(true);

  climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  public void setClimberUp() {

    climberPID.setReference(climberUpPosition, ControlType.kPosition);

  }

  public void climb(){

    climberPID.setReference(climberDownPosition, ControlType.kPosition);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
