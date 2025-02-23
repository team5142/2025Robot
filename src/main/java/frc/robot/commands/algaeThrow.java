package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.PositionClass.Positions;
import frc.robot.subsystems.IntakeSubsystem;
public class algaeThrow extends SequentialCommandGroup{

    double delay = 1; //amount of time to wait before throwing algae

    public algaeThrow(){
        
        addRequirements(RobotContainer.intake);

    addCommands(

        new moveToPosition(Positions.L4), //while moving to l4 (top)...
        new WaitCommand(delay), // wait a small amount of time until we're close to the top
        new InstantCommand(RobotContainer.intake::ejectAlgae) //throw the algae with upwards momentum
            
        //note: maybe also starting with arm down all the way -> flinging it up would make it have more momentum?

         );
    }
}