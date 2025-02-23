// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants.CurrentLimits;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PositionClass.Positions;

public class ArmSubsystem extends SubsystemBase {

  private SparkMax armMotor;
  private SparkMaxConfig armConfig;
  private SparkClosedLoopController armPID;
  private AbsoluteEncoder armAbsoluteEncoder;
  private RelativeEncoder armRelativeEncoder;

  private final double kP = 0.1;
  private final double kI = 0;
  private final double kD = 0;

  private final double kMax = 0.25;
  private final double kMin = -0.25;

  private final double armRatio = (4 * 3 * (65/15)); //gear ratio from the relative to absolute encoder

  /** Creates a new IntakeSubsystem. */
  public ArmSubsystem() {

    armMotor = new SparkMax(18, MotorType.kBrushless);

    armConfig = new SparkMaxConfig();

    armPID = armMotor.getClosedLoopController();

    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();

    configureArmMotor();

    armRelativeEncoder = armMotor.getEncoder();

    armPID.setReference(Positions.Home.armPosition, ControlType.kPosition);

    armRelativeEncoder.setPosition(armAbsoluteEncoder.getPosition()); // This is to set the relative encoder to the absolute encoder's position

  }

  private void configureArmMotor(){
    //In case the spark max needs to be replaced, this method is useful.
    //It configures the motor by adding to the intakeConfig object, 
    //and then applies it, and at the same time resets parameters.
    //NOTE: For PID Testing, we might need to change the persist mode.
  
  armConfig.softLimit
  .forwardSoftLimit(34)
  .reverseSoftLimit(0);

  armConfig.closedLoop
  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  .p(kP)
  .i(kI)
  .d(kD)
  .outputRange(kMin, kMax);

  armConfig.absoluteEncoder.positionConversionFactor(armRatio);



  armConfig.smartCurrentLimit(CurrentLimits.Neo550)
  .inverted(true);
  
  armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setArmPID(double position){
    armPID.setReference(position, ControlType.kPosition);
  }

  public void setArmPosition(Positions Position){
    armPID.setReference(Position.armPosition, ControlType.kPosition);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
