// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.CurrentLimits;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.PositionClass.Positions;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */

  private SparkMax leadElevatorMotor;
  private SparkMax followingElevatorMotor;
  private SparkMax secondaryElevatorMotor;

  private SparkMaxConfig leadElevatorConfig;
  private SparkMaxConfig followingElevatorConfig;
  private SparkMaxConfig secondaryElevatorConfig;

  private DigitalInput primaryLimitSwitch;
  // private DigitalInput secondaryLimitSwitch;
  private final double primarykP = 0.1;
  private final double primarykI = 0;
  private final double primarykD = 0;

  private final double primaryForwardSpeedLimit = 1;
  private final double primaryReverseSpeedLimit = -1;

  private final double primaryResetSpeed = -0.25;

  private final double secondarykP = 0.25;
  private final double secondarykI = 0;
  private final double secondarykD = 0;

  private final double secondaryForwardSpeedLimit = 1;
  private final double secondaryReverseSpeedLimit = -1;

  private final double secondaryResetSpeed = -0.25;

  private SparkClosedLoopController leadPID;
  private SparkClosedLoopController secondaryPID;


  /** Creates a new ClimberSubsystem. */
  public ElevatorSubsystem() {


    leadElevatorMotor = new SparkMax(15, MotorType.kBrushless);
    followingElevatorMotor = new SparkMax(16, MotorType.kBrushless);
    //These two control the main stage
    secondaryElevatorMotor = new SparkMax(17, MotorType.kBrushless);
    //This motor controls the second stage

    primaryLimitSwitch = new DigitalInput(0);
    // secondaryLimitSwitch = new DigitalInput(1);

    

    //Initialize PIDS
    leadPID = leadElevatorMotor.getClosedLoopController();
    secondaryPID = secondaryElevatorMotor.getClosedLoopController();


    leadElevatorConfig = new SparkMaxConfig();
    followingElevatorConfig = new SparkMaxConfig();
    secondaryElevatorConfig = new SparkMaxConfig();
    //Only one config is necessary as it will be used for all the canandmags.
   

    configureElevatorMotors();


   

  }

  private void configureElevatorMotors(){

    //Encoders are reset, if we want to go by inches or something we can try multiplying but
    //I think it's easier to just go by rotations and find values with hardware client

    followingElevatorConfig.follow(15, true); 
      //Follows the lead motor, invert is set to true. We may want to also invert the other motor.

    leadElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500);
    followingElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500);
    secondaryElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500)
    .inverted(true);
      //Applies a 40 amp limit

    leadElevatorConfig.idleMode(IdleMode.kBrake)
    .softLimit
    .forwardSoftLimit(190)
    .reverseSoftLimit(0);
    
    followingElevatorConfig.idleMode(IdleMode.kBrake);
  
    secondaryElevatorConfig.idleMode(IdleMode.kBrake)
    .softLimit
    .forwardSoftLimit(140)
    .reverseSoftLimit(0);
      // Makes it so that the motors stay in their position after being shut off.

    
    leadElevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(primarykP)
      .i(primarykI)
      .d(primarykD)
      .outputRange(primaryReverseSpeedLimit, primaryForwardSpeedLimit);

    secondaryElevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(secondarykP)
      .i(secondarykI)
      .d(secondarykD)
      .outputRange(secondaryReverseSpeedLimit, secondaryForwardSpeedLimit);


    leadElevatorMotor.configure(leadElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingElevatorMotor.configure(followingElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    secondaryElevatorMotor.configure(secondaryElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //Resets and configures sparkmaxes


   
  }

  public boolean isElevatorActive(){

    return secondaryElevatorMotor.getEncoder().getPosition() > 20; //used to tell if elevator encoder is higher than 20

  }

  public boolean isPrimaryLimitSwitchPressed(){

    return primaryLimitSwitch.get();

  }

  
  public void setPrimaryPID(double height) {

    leadPID.setReference(height, ControlType.kPosition);

  }

  public void setSecondaryPID(double height) {

    secondaryPID.setReference(height, ControlType.kPosition);
    
  }

  public void setPrimaryPosition(Positions position) {

    leadPID.setReference(position.primaryElevator, ControlType.kPosition);

  }

  public void setSecondaryPosition(Positions position) {

    secondaryPID.setReference(position.secondaryElevator, ControlType.kPosition);
    
  }

  public void primaryForward(){

    leadElevatorMotor.set(1);

  }

  public void primaryDown(){

    leadElevatorMotor.set(primaryResetSpeed);

  }

  public void primaryStop(){

    leadElevatorMotor.set(0);

  }

  public void zeroPrimaryEncoder(){

    leadElevatorMotor.getEncoder().setPosition(0);

  }
  
  public void turnOffBrake(){

    leadElevatorConfig.idleMode(IdleMode.kCoast);
    followingElevatorConfig.idleMode(IdleMode.kCoast);
    secondaryElevatorConfig.idleMode(IdleMode.kCoast);

    leadElevatorMotor.configure(leadElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingElevatorMotor.configure(followingElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    secondaryElevatorMotor.configure(secondaryElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }
  
  public void turnOnBrake(){

    leadElevatorConfig.idleMode(IdleMode.kBrake);
    followingElevatorConfig.idleMode(IdleMode.kBrake);
    secondaryElevatorConfig.idleMode(IdleMode.kBrake);  

    leadElevatorMotor.configure(leadElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingElevatorMotor.configure(followingElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    secondaryElevatorMotor.configure(secondaryElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // display isAlgaeIntaked() and left and right coral to the SmartDashboard

    SmartDashboard.putBoolean("elevator up", isElevatorActive());
    SmartDashboard.putBoolean("switch pressed", isPrimaryLimitSwitchPressed());



  }
}