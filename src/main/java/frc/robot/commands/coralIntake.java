package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.PositionClass.Positions;
import frc.robot.RobotContainer;

public class coralIntake extends SequentialCommandGroup{


    public coralIntake(){

        addRequirements(RobotContainer.intake, RobotContainer.led);

    addCommands(

        new moveToPosition(Positions.Feed),
        new InstantCommand(RobotContainer.intake::intakeCoral),
        new WaitUntilCommand(RobotContainer.intake::isCoralIntaked), //wait until the sensor senses on
        new WaitUntilCommand(RobotContainer.intake::isNeitherCoralIntaked), //wait until it passes the sensor

        new InstantCommand(RobotContainer.intake::stopCoral),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(RobotContainer.led::setBothStrobeRed),
                new WaitCommand(1),
                new InstantCommand(RobotContainer.led::setBothRed)
            ),
            new xboxVibrate()
        )
        
        
        
    );
    }
}