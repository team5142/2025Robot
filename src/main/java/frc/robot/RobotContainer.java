// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.List;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PoseClass.Poses;
import frc.robot.Constants.PositionClass.Positions;
import frc.robot.commands.turnToAngle;
import frc.robot.commands.algaeIntake;
import frc.robot.commands.algaeThrow;

import frc.robot.commands.coralIntake;
import frc.robot.commands.moveToPosition;
import frc.robot.commands.xboxVibrate;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.autoCoralIntake;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* ============== Make a function to do this below */

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband This should be 0.001 for cubed controls
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    //DEFINE CONTROLLERS
    //main drive controller
    public final static CommandXboxController joystick = new CommandXboxController(0);
    //right side of button box: port 1    
    private final CommandGenericHID rightSide = new CommandGenericHID(1);
    //left side of button box: port 2
    private final CommandGenericHID leftSide = new CommandGenericHID(2);
    
    public final static ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final static ArmSubsystem arm = new ArmSubsystem();
    public final static IntakeSubsystem intake = new IntakeSubsystem();
    public final static SmartDashboardSubsystem smartdashboard = new SmartDashboardSubsystem();
    public final static LEDSubsystem led = new LEDSubsystem();
    public final static ClimberSubsystem climber = new ClimberSubsystem();
    private final SendableChooser<Command> autoChooser;

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    public RobotContainer() {

        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        CameraServer.startAutomaticCapture();
        drivetrain.setIMU180(); //we start the match facing backwards so we need to set the imu reversed

    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according toc  WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)  /2 is to slow it down
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );     



        joystick.rightBumper().onTrue
        (Commands.runOnce(intake::ejectCoral))//shoots out the coral, or just manual intake 
                          .onFalse(Commands.runOnce(intake::stopCoral));
                        //   .andThen(new moveToPosition(Positions.Home)));

        rightSide.button(7).onTrue(new moveToPosition(Positions.L1));
        rightSide.button(3).onTrue(new moveToPosition(Positions.L2));
        rightSide.button(2).onTrue(new moveToPosition(Positions.L3));
        rightSide.button(1).onTrue(new moveToPosition(Positions.L4));
        rightSide.button(8).onTrue(new moveToPosition(Positions.Home));

        rightSide.button(5).onTrue(new moveToPosition(Positions.Algae1));
        rightSide.button(4).onTrue(new moveToPosition(Positions.Algae2));

        rightSide.button(9).onTrue(Commands.runOnce(climber::setClimberUp));
        rightSide.button(10).onTrue(Commands.runOnce(() -> {climber.climb(); led.setBothStrobeRed();}));

        leftSide.button(3).onTrue(new SequentialCommandGroup( //Algae intake

        new moveToPosition(Positions.groundAlgae).unless(elevator::isElevatorActive),
        //if the elevator is active, just intake from the reef. If it isn't, we want to ground intake, so put the arm down.
        new algaeIntake().handleInterrupt(() -> {intake.stopAlgae(); intake.turnOffAlgaeLight();}).withTimeout(8)
        //intake an algae, if it doesn't work within 8 seconds stop (handle interrupt detects the timeout).

        )); 

        leftSide.button(1).onTrue(new moveToPosition(Positions.Processor));


        //intake left
        leftSide.button(6).onTrue(
          
        new ConditionalCommand(
          
          Commands.sequence( //on true
            Commands.runOnce(() -> {led.setLeftRed(); led.setRightOff();}),
            new moveToPosition(Positions.Home), //if elevator is up and intake is pressed, go to home first
            new WaitCommand(0.5),
            new coralIntake().handleInterrupt(() -> joystick.setRumble(RumbleType.kBothRumble, 0)).withTimeout(10),
            Commands.runOnce(intake::stopCoral),
            new moveToPosition(Positions.Intaked)), 
          
          Commands.sequence( //on false
            Commands.runOnce(() -> {led.setLeftRed(); led.setRightOff();}),
            new coralIntake().handleInterrupt(() -> joystick.setRumble(RumbleType.kBothRumble, 0)).withTimeout(10),
            Commands.runOnce(intake::stopCoral),
            new moveToPosition(Positions.Intaked)), 
          
          elevator::isElevatorActive)); //condition
          
        //intake right
        leftSide.button(4).onTrue(
          
        new ConditionalCommand(
        
          Commands.sequence(
            Commands.runOnce(() -> {led.setRightRed(); led.setLeftOff();}),
            new moveToPosition(Positions.Home), //if elevator is up and intake is pressed, go to home first
            new WaitCommand(0.5),
            new coralIntake().handleInterrupt(() -> joystick.setRumble(RumbleType.kBothRumble, 0)).withTimeout(10),
            Commands.runOnce(intake::stopCoral),
            new moveToPosition(Positions.Intaked)), 
          
          Commands.sequence(
            Commands.runOnce(() -> {led.setRightRed(); led.setLeftOff();}),
            new coralIntake().handleInterrupt(() -> joystick.setRumble(RumbleType.kBothRumble, 0)).withTimeout(10),
            Commands.runOnce(intake::stopCoral),
            new moveToPosition(Positions.Intaked)), 
            
          elevator::isElevatorActive));


       

        rightSide.button(6).onTrue(new algaeThrow()); //throws algae with upward momentum while going to top position
        leftSide.button(2).onTrue(Commands.runOnce(intake::ejectAlgae))
        .onFalse(Commands.runOnce(intake::stopAlgae).andThen(Commands.runOnce(intake::turnOffAlgaeLight)));

        leftSide.button(5).whileTrue( //ground coral intake
          Commands.runOnce(intake::intakeAlgae)
          .alongWith(new moveToPosition(Positions.groundCoral)))

          .onFalse(Commands.runOnce(intake::holdAlgae)
          .andThen(new WaitCommand(0.5))
          .andThen(new moveToPosition(Positions.Home)));

        leftSide.button(7).onTrue(

        Commands.sequence(

         Commands.runOnce(led::refreshLEDs), //sets to 5v strip if it gets messed up
         new WaitCommand(0.1),
         Commands.runOnce(led::setBothOff),
         new WaitCommand(0.1),
         Commands.runOnce(led::setBothLava)

        ));
        
       
        

        joystick.x().whileTrue(new turnToAngle(120)); //TODO: make this be correct for -180 to 180
        joystick.y().whileTrue(new turnToAngle(180));
        joystick.b().whileTrue(new turnToAngle(240));
            
        
        
   

        joystick.a().whileTrue(Commands.defer(() -> AutoBuilder.followPath(inferPath()), Set.of(drivetrain)).unless(() -> (LimelightHelpers.getFiducialID("limelight-front") == -1))); // on a press run the pathplanner path inferred by limelight, from pose gotten from limelight unless there is no id visible
        

          // joystick.a().whileTrue(followPathCommand("center"));
       
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // plug flash drive in for these?
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //rightSide.button(9).onTrue(Commands.runOnce(SignalLogger::start));
        //rightSide.button(10).onTrue(Commands.runOnce(SignalLogger::stop));


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.rightBumper().onTrue(Commands.runOnce(() -> {intake.ejectCoral(); intake.ejectAlgae();})) //run both intakes from right trigger
        .onFalse(Commands.runOnce(() -> {intake.stopCoral(); intake.stopAlgae();})); //stop both intakes from right trigger release

        

        rightSide.button(11).onTrue(Commands.runOnce(led::setBothParty));
        rightSide.button(12).onTrue(Commands.runOnce(led::setBothLava));
        leftSide.button(9).onTrue(Commands.runOnce(led::setBothScanner));
        leftSide.button(10).onTrue(Commands.runOnce(led::setBothFire));




        // End of button binds


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public PathPlannerPath inferPath() {

        Pose2d targetPose = inferDesiredPose();

        Rotation2d desiredHeading = Rotation2d.fromDegrees(LimelightHelpers.getTX("limelight-front")).minus(Rotation2d.fromDegrees(drivetrain.getRotation().getDegrees())).times(-1);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
     
        new Pose2d(
        drivetrain.getPose().getX(), 
        drivetrain.getPose().getY(), 
        desiredHeading), //starting pose: current x and y plus current heading (direction of travel),
 
        new Pose2d(
        targetPose.getX(),
        targetPose.getY(),
        desiredHeading))

        ;

        // List<Waypoint> goNowhere = PathPlannerPath.waypointsFromPoses(
        //   new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
        //   new Pose2d(0, 0 ,Rotation2d.fromDegrees(0))
       
        // );

        PathConstraints constraints = new PathConstraints(2.5, 2.5, 3 * Math.PI, 4 * Math.PI); // converted degrees to radians, everything taken from pathplanner settings
        SmartDashboard.putNumber("GoalX", targetPose.getX());
        SmartDashboard.putNumber("GoalY", targetPose.getY());
        SmartDashboard.putNumber("Goal Heading", desiredHeading.getDegrees());
        SmartDashboard.putNumber("GoalRotation", targetPose.getRotation().getDegrees());

        if (LimelightHelpers.getFiducialID("limelight-front") == -1) {

        PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        path.preventFlipping = true;
        return path;    

        }



        return new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );    
    }

    public Pose2d inferDesiredPose() {


        switch ((int)LimelightHelpers.getFiducialID("limelight-front")) { //get the apriltag id from the limelight, and use the giant switch statement to determine the desired pose

        case 7:
          return FlippingUtil.flipFieldPose(Poses.AB.desiredPose); //for red tags flip the pose we found with pathplanner to the other side
        case 18:
          return Poses.AB.desiredPose;
        case 8:
          return FlippingUtil.flipFieldPose(Poses.CD.desiredPose);
        case 17:
          return Poses.CD.desiredPose;
        case 9:
          return FlippingUtil.flipFieldPose(Poses.EF.desiredPose);
        case 22:
          return Poses.EF.desiredPose;
        case 10:
          return FlippingUtil.flipFieldPose(Poses.GH.desiredPose);
        case 21:
          return Poses.GH.desiredPose;
        case 11:
          return FlippingUtil.flipFieldPose(Poses.IJ.desiredPose);
        case 20:
          return Poses.IJ.desiredPose;
        case 6:
          return FlippingUtil.flipFieldPose(Poses.KL.desiredPose);
        case 19:
          return Poses.KL.desiredPose;
        case 2:
          return FlippingUtil.flipFieldPose(Poses.rightCoralStation.desiredPose);
        case 12:
          return Poses.rightCoralStation.desiredPose;
        case 1:
          return FlippingUtil.flipFieldPose(Poses.leftCoralStation.desiredPose);
        case 13:
          return Poses.leftCoralStation.desiredPose;
        default:
          return RobotContainer.drivetrain.getPose(); // Return current position if no apriltag matches
        
      }

    }    

    public Command followPathCommand(String pathName) {

    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);

    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }



    public void registerNamedCommands() { //registering commands for pathplanner autos

        NamedCommands.registerCommand("algaeIntake", new algaeIntake().withTimeout(8));
        NamedCommands.registerCommand("algaeThrow", new algaeThrow());
        NamedCommands.registerCommand("moveToHome", new moveToPosition(Positions.Home));
        NamedCommands.registerCommand("moveToL1", new moveToPosition(Positions.L1));
        NamedCommands.registerCommand("moveToL2", new moveToPosition(Positions.L2));
        NamedCommands.registerCommand("moveToL3", new moveToPosition(Positions.L3));
        NamedCommands.registerCommand("moveToL4", new moveToPosition(Positions.L4).andThen(new WaitCommand(1)));
        NamedCommands.registerCommand("autoCoralIntake", new autoCoralIntake());
        NamedCommands.registerCommand("coralIntake", //paste of previous coral intake command
        new coralIntake().withTimeout(6).unless(elevator::isElevatorActive)
                         .andThen(new moveToPosition(Positions.Intaked))
                         .andThen(Commands.runOnce(intake::stopCoral)));

        NamedCommands.registerCommand("coralShoot", 

        Commands.sequence(//for shooting out coral autonomously:

            Commands.runOnce(intake::ejectCoral), //run the intake to shoot it out
            new WaitCommand(1), //wait a bit (change this to make it take less long)
            Commands.runOnce(intake::stopCoral), //stop the intake
            new moveToPosition(Positions.Home) //elevator down

        ));
    }

    public Command getAutonomousCommand() {

        return autoChooser.getSelected();

    }
}
