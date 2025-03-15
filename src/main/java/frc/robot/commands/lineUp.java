package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.PoseClass.Poses;

public class lineUp extends Command {

    private Pose2d targetPose;
    private PathPlannerPath path;
    private FlippingUtil flippingUtil = new FlippingUtil();

    public lineUp() {

        // Require the drivetrain subsystem

        addRequirements(RobotContainer.drivetrain);
        
    }

    @Override
    public void initialize() {
        // Reset the PID controller to ensure it starts fresh
        targetPose = inferDesiredPose();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        RobotContainer.drivetrain.getPose(),
        targetPose
        );

        PathConstraints constraints = new PathConstraints(1.75, 1.75, 3 * Math.PI, 4 * Math.PI); // converted degrees to radians, everything taken from pathplanner settings
        
        path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
);
    }

    private Pose2d inferDesiredPose() {

      switch ((int)LimelightHelpers.getFiducialID("front")) { //get the apriltag id from the limelight, and using the giant switch statement to determine the desired pose

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
    @Override
    public void execute() {

        AutoBuilder.followPath(path);
        
    }

    @Override
    public boolean isFinished() {

        return false; //do we need this for auto?

    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        RobotContainer.drivetrain.setControl(new SwerveRequest.Idle());
    }
}
