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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimits;

public class LEDSubsystem extends SubsystemBase {


  private static Spark leftBlinkin;
  private static Spark rightBlinkin;
 
  /** Creates a new ClimberSubsystem. */

  public LEDSubsystem() {

   leftBlinkin = new Spark(0);
   rightBlinkin = new Spark(1);

  }

  public void setRightRed(){

    rightBlinkin.set(-0.17); //breath red value

  }

  public void setLeftRed(){

    leftBlinkin.set(-0.17); //breath red value
    
  }

  public void setRightGreen(){

    rightBlinkin.set(0.73); //red

  }
  
  public void setLeftGreen(){
    
    leftBlinkin.set(0.73); //red

  }

  public void setRightOff(){

    rightBlinkin.set(0.99);

  }
  
  public void setLeftOff(){
    
    leftBlinkin.set(0.99);

  }


}