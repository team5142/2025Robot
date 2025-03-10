package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
public class xboxVibrate extends SequentialCommandGroup{

    double length = 0.25; //rumble length

    public xboxVibrate() {
        

    addCommands(

        new InstantCommand(() -> RobotContainer.joystick.setRumble(RumbleType.kBothRumble, 1)),
        new WaitCommand(length),
        new InstantCommand(() -> RobotContainer.joystick.setRumble(RumbleType.kBothRumble, 0))

         );
    }
}