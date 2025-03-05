package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class turnToAngle extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private PIDController pidController;
    private Rotation2d targetAngle;
    private SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    public turnToAngle(CommandSwerveDrivetrain drivetrain, Rotation2d tAngle) {
        this.drivetrain = drivetrain;
        this.targetAngle = tAngle;
        SmartDashboard.putNumber("tangle", tAngle.getDegrees());
        // Initialize PID controller with gains (kP, kI, kD)
        this.pidController = new PIDController(0.02, 0.0, 0.0); // Tune these values
        pidController.setTolerance(1.0); // Tolerance in degrees

        // Require the drivetrain subsystem
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset the PID controller to ensure it starts fresh
        pidController.reset();
    }

    @Override
    public void execute() {
        // Get the current heading from the drivetrain
        Rotation2d currentHeading = drivetrain.getHeading();

        // Calculate the rotation output using the PID controller
        double rotationOutput = pidController.calculate(currentHeading.getDegrees(), targetAngle.getDegrees());

        // Apply the rotation output to the drivetrain using a SwerveRequest
        drivetrain.setControl(driveRequest.withRotationalRate(rotationOutput));

        // Optional: Log the values to SmartDashboard for debugging
        SmartDashboard.putNumber("Current Heading", currentHeading.getDegrees());
        SmartDashboard.putNumber("Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Rotation Output", rotationOutput);
    }

    @Override
    public boolean isFinished() {
        // End the command when the PID controller reaches the setpoint
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}