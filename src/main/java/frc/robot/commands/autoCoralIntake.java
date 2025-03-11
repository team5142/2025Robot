package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.PositionClass.Positions;
import frc.robot.RobotContainer;
public class autoCoralIntake extends SequentialCommandGroup{


    public autoCoralIntake(){

        addRequirements(RobotContainer.intake, RobotContainer.led);

    addCommands(
        new InstantCommand(RobotContainer.led::setRightRed),
        new InstantCommand(RobotContainer.led::setLeftOff),
        new moveToPosition(Positions.Feed),
        new InstantCommand(RobotContainer.intake::intakeCoral),
        new WaitUntilCommand(RobotContainer.intake::isCoralIntaked),
        new WaitUntilCommand(() -> RobotContainer.intake.isNeitherCoralIntaked()), //wait until it passes the sensor
        new InstantCommand(RobotContainer.intake::stopCoral),
        new InstantCommand(RobotContainer.led::setBothRed)
    );
    }
}