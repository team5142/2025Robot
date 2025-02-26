package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.PositionClass.Positions;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {

    public String updatedPosition;

    public SmartDashboardSubsystem() {}

    public void updatePosition(Positions position){

        updatedPosition = position.label;

    }

    public void refreshOdometryDisplay() {
        
    }
    public void refreshPoseDisplay(){
        
    }
    public void refreshIMUDisplay() {
        
    }
    public void refreshElevatorPositioningDisplay(){

        SmartDashboard.putString("Position :", updatedPosition);

    }
    public void refreshArmRotationDisplay(){
        
    }
    public void refreshIntakeDisplay(){

        SmartDashboard.putBoolean("Coral Intaked :", RobotContainer.intake.isCoralIntaked());

        SmartDashboard.putBoolean("Algae Intaked", RobotContainer.intake.isAlgaeIntaked());

    }

    public void refreshClimberDisplay(){
        
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        refreshOdometryDisplay();
        refreshPoseDisplay();
        refreshIMUDisplay();
        refreshElevatorPositioningDisplay();
        refreshArmRotationDisplay();
        refreshIntakeDisplay();
        refreshClimberDisplay();
    }
}
