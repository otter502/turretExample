// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kStandardDt = 0.02;

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class TurretConstants{

        public static final int kMotorID = 1;
        public static final double kGearRatio = 58.2;
        
        //conversion rate, (motor rotations) * conversionRate = turret rate
        private static final EncoderConfig kEncoderConfig = new EncoderConfig()
            .positionConversionFactor(kGearRatio)
            .velocityConversionFactor(kGearRatio);
        
        private static final ClosedLoopConfig kCLConfig = new ClosedLoopConfig()
            .p(0.1).i(0).d(0)
            .positionWrappingEnabled(true);
        
        public static final SparkBaseConfig kMotorConfig = new SparkMaxConfig()
            .apply(kEncoderConfig)
            .apply(kCLConfig)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(35)
            .secondaryCurrentLimit(40);
        
        public static final MotorType kMotorType = MotorType.kBrushless;
        
        public static final SimpleMotorFeedforward kFeedForwardCalc = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        
        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(2.0, 1.0); //units are the output units (degrees of the turret in this example)
        
        public static final Rotation2d kStartingAngle = Rotation2d.fromDegrees(0.0);

        public static final SingleJointedArmSim kSim = new SingleJointedArmSim(DCMotor.getNEO(1), kGearRatio, 1, 1, -100, 100, false, 0.0);

    }
}
