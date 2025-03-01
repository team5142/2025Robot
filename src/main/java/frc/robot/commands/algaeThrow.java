package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.PositionClass.Positions;
public class algaeThrow extends SequentialCommandGroup{

    double armDelay = 1; //amount of time to wait before raising arm and first stage
    double throwDelay = 1; //amount of time to wait before throwing algae


    public algaeThrow(){
        
        addRequirements(RobotContainer.intake);

    addCommands(

            //NOTE: The second stage hits the top first. In order to get the most launch,
            //we need to stall the first stage
        new moveToPosition(Positions.Barge), //put down arm and go up only a little with first stage, as much as possible second stage
        new WaitCommand(armDelay), // wait a small amount of time until we're close to the top of second stage
        new moveToPosition(Positions.L4), //now still go to top but fling arm up and first stage
        new WaitCommand(throwDelay), //wait until we are at the top to eject

        new InstantCommand(RobotContainer.intake::ejectAlgae),
        new WaitCommand(1),
        new InstantCommand(RobotContainer.intake::stopAlgae) //throw the algae with upwards momentum
            

         );
    }
}