// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.Constants.CurrentLimits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSubsystem extends SubsystemBase {

  private Canandcolor leftCoralSensor;
  private Canandcolor rightCoralSensor;
  private Canandcolor algaeSensor;

  private final double algaeGThreshold = 0.40;
  private final double coralProximityThreshold = 0.05;

  private SparkMax coralMotor;
  private SparkMaxConfig coralConfig;
  
  private SparkMax algaeMotor;
  private SparkMaxConfig algaeConfig;

  private final double algaekP = 0.01;
  private final double algaekI = 0.01;
  private final double algaekD = 0.01;

  private final double algaeFF = Constants.FeedForwards.Neo550FF;

  private final double algaeForwardSpeedLimit = 0.5;
  private final double algaeReverseSpeedLimit = -0.5;

  private final double algaeIntakeSpeed = 1;
  private final double algaeEjectSpeed = -1;
  private final double algaeHoldSpeed = 0.15;

  private final double coralIntakeSpeed = 0.225;
  private final double coralSetSpeed = -0.15;

  
  private final double coralEjectSpeed = 1; 


  private RelativeEncoder algaeEncoder;

  private SparkClosedLoopController algaePID; //velocity PID to keep algae in

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    coralMotor = new SparkMax(19, MotorType.kBrushless);
    algaeMotor = new SparkMax(20, MotorType.kBrushless);

    coralConfig = new SparkMaxConfig();
    algaeConfig = new SparkMaxConfig();

    algaeEncoder = algaeMotor.getEncoder();

    algaePID = algaeMotor.getClosedLoopController();

    configureIntakeMotors();

    rightCoralSensor = new Canandcolor(22);
    leftCoralSensor = new Canandcolor(23);
    algaeSensor = new Canandcolor(24);

    turnOffAlgaeLight();

  }

  private void configureIntakeMotors(){
    //In case the spark max needs to be replaced, this method is useful.
    //It configures the motor by adding to the intakeConfig object, 
    //and then applies it, and at the same time resets parameters.
    //NOTE: For PID Testing, we might need to change the persist mode.

    coralConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);
    algaeConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    coralConfig.smartCurrentLimit(CurrentLimits.Neo550)
    .idleMode(IdleMode.kBrake);

    algaeConfig.smartCurrentLimit(CurrentLimits.Neo550)
    .idleMode(IdleMode.kBrake);
  
    algaeConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(algaekP)
      .i(algaekI)
      .d(algaekD)
      .velocityFF(algaeFF)
      .outputRange(algaeReverseSpeedLimit, algaeForwardSpeedLimit);

    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  //methods to be called in commands and as instant commands

  
  
  public void holdAlgae(){

    // algaePID.setReference(algaeHoldSpeed, ControlType.kVelocity);
    algaeMotor.set(algaeHoldSpeed);
  
    //keeps a vpid velocity to hold the ball in tight
  }



  public void intakeAlgae(){
    algaeMotor.set(algaeIntakeSpeed);
  }

  public void ejectAlgae(){
    algaeMotor.set(algaeEjectSpeed);
  }

  public void stopAlgae(){
    algaeMotor.set(0);
  }

  public void intakeCoral(){
    coralMotor.set(coralIntakeSpeed);
  }

  public void setCoral(){
    coralMotor.set(coralSetSpeed);
  }

  public void ejectCoral(){
    coralMotor.set(coralEjectSpeed);
  }

  public void stopCoral(){
    coralMotor.set(0);
  }

  public boolean isAlgaeIntaked(){

    return (algaeSensor.getGreen() > algaeGThreshold);

    //algae sensor returns a value between 0 and 1, and we will find the value that determines if the algae is present.
  }

 
  public boolean isLeftCoralIntaked(){
    return leftCoralSensor.getProximity() < coralProximityThreshold;
  }

  public boolean isRightCoralIntaked(){
    return rightCoralSensor.getProximity() < coralProximityThreshold;
  }

  public boolean isCoralIntaked(){

    return (isLeftCoralIntaked() || isRightCoralIntaked());
    
  }

  public boolean isNeitherCoralIntaked(){

    return (!isLeftCoralIntaked() && !isRightCoralIntaked());
    
  }

  public void turnOffBrake(){

    coralConfig.idleMode(IdleMode.kCoast);
    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  
  public void turnOnBrake(){

    coralConfig.idleMode(IdleMode.kBrake);
    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void turnOffAlgaeLight(){

    algaeSensor.setLampLEDBrightness(0);

  }

  
  public void turnOnAlgaeLight(){

    algaeSensor.setLampLEDBrightness(0.99);
    
  }
  public double intakePos;

  public void setIntakePos(){
     intakePos = coralMotor.getEncoder().getPosition();
  }

  public boolean isCoralReady(){
    return intakePos + 10 < coralMotor.getEncoder().getPosition();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // display isAlgaeIntaked() and left and right coral to the SmartDashboard
    // SmartDashboard.putBoolean("LEFT CORAL LOADED:", isLeftCoralIntaked());
    // SmartDashboard.putBoolean("ALGAE LOADED:", isAlgaeIntaked());
    // SmartDashboard.putBoolean("RIGHT CORAL LOADED:", isRightCoralIntaked());
  }
}
