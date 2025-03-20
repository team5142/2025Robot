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
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimits;
import edu.wpi.first.wpilibj.PWM;

public class LEDSubsystem extends SubsystemBase {


  private static PWM leftBlinkin;
  private static PWM rightBlinkin;
 
  /** Creates a new LEDSubsystem. */

  public LEDSubsystem() {

   leftBlinkin = new PWM(0);
   rightBlinkin = new PWM(1);

   refreshLEDs(); // from rev blinkin troubleshooting page, leds kept switching from 5v to 12v strip on their own

   setBothLava();
   
  }

  public void setRightRed(){

    rightBlinkin.setPulseTimeMicroseconds(1805); //red value

  }

  public void setLeftRed(){

    leftBlinkin.setPulseTimeMicroseconds(1805); //red value
    
  }

  public void setRightGreen(){

    rightBlinkin.setPulseTimeMicroseconds(1885); //green value

  }
  
  public void setLeftGreen(){
    
    leftBlinkin.setPulseTimeMicroseconds(1885); //green value

  }

  public void setRightOff(){

    rightBlinkin.setPulseTimeMicroseconds(1995);

  }
  
  public void setLeftOff(){
    
    leftBlinkin.setPulseTimeMicroseconds(1995);

  }

  public void setBothRed() {

    setRightRed();
    setLeftRed();
    
  }

  public void setBothStrobeRed(){

    leftBlinkin.setPulseTimeMicroseconds(1445);
    rightBlinkin.setPulseTimeMicroseconds(1445);

  }

  public void setBothGreen() {
    setRightGreen();
    setLeftGreen();
  }

  public void setBothOff() {
    setRightOff();
    setLeftOff();
  }

  public void setBothParty(){

    leftBlinkin.setPulseTimeMicroseconds(1015);
    rightBlinkin.setPulseTimeMicroseconds(1015);

  }

  public void setBothFire(){

    leftBlinkin.setPulseTimeMicroseconds(1255);
    rightBlinkin.setPulseTimeMicroseconds(1255);

  }

  public void setBothScanner(){

    leftBlinkin.setPulseTimeMicroseconds(1325);
    rightBlinkin.setPulseTimeMicroseconds(1325);

  }
  
  public void setBothLava(){

    leftBlinkin.setPulseTimeMicroseconds(1305);
    rightBlinkin.setPulseTimeMicroseconds(1305);

  }

  public void refreshLEDs(){

    leftBlinkin.setPulseTimeMicroseconds(2125); //makes it go to 5v strip
    rightBlinkin.setPulseTimeMicroseconds(2125);

  }


}