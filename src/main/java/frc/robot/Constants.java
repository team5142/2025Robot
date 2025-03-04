// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}


	public static final class PositionClass {
		// Positions in order: Primary Elevator, Secondary Elevator, Arm Position (0 -> 1)
		public enum Positions {
			Home("Home",
							0,
							0,
							1.7),
			Feed("Feed",
							0,
							0,
							0),
			L1("L1",
							20,
							0,
							1.7),
			L2("L2",
							46,
							0,
							1.7),
			L3("L3",
							67,
							39,
							1.7),
			L4("L4",
							90,
							114,
							4),
			Algae1("Algae1",
							37,
							0,
							6),
			Algae2("Algae2",
							62,
							39,
							6),
			Processor("Processor",
							0,
							0,
							13),
			BargePrep("Barge",
							80,
							108,
							1.7),
			groundAlgae("groundAlgae",
							0,
							0,
							24);

			public final String label;
			public final double primaryElevator;
			public final double secondaryElevator;
			public final double armPosition;

			private Positions(String label,
							double primaryElevator,
							double secondaryElevator,
							double armPosition) {
					this.label = label;
					this.primaryElevator = primaryElevator;
					this.secondaryElevator = secondaryElevator;
					this.armPosition = armPosition;
			}

		}
    }


	public static final class CurrentLimits {

		public static final int Neo550 = 20;
		public static final int Neo500 = 40;
	}

	public static final class FeedForwards {

		public static final double Neo500FF = (1/473);
		public static final double Neo550FF = (1/917);

	}
	






	public static final class VisionConstants {

		

		public static final class LimeLightConstants {

			// If changing this value, do not forget to set it in LL
			public static final String LLAprilTagName = "limelight-at"; // Limelight that will track Apriltags; may
																		// decide to use multiple ones

			// centerpose from BLUE coordinate system
			public static final Pose2d centerFieldPose = new Pose2d(8.308467, 4.098925, new Rotation2d(0));
			// NEW origin from the old origin point of view in the old coordiinate system
			public static final Pose2d originFieldPose = new Pose2d(-8.308467, -4.098925, new Rotation2d(0));

			// *** LL Detector ***
			public static final String LLDetectorName = "limelight-d"; // Limelight that will track Apriltags; may
																		// decide to use multiple ones

			// TODO: measure and verify this transform
			public static final Transform2d cameraToRobotTransform = new Transform2d(new Translation2d(-0.30, -0.23),
					Rotation2d.fromDegrees(180));

		}

	}

}
