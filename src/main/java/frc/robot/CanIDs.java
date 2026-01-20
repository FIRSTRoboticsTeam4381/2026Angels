// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

/** Constants file for CAN IDs */
public final class CanIDs {

    // Map of CAN IDs for URCL
    // Keep this up to date with the actual CAN IDs so logs make sense!
    public static final Map<Integer, String> URCL_IDS = Map.ofEntries(
          Map.entry(10, "Swerve/FL/Drive"),
          Map.entry(11, "Swerve/FL/Angle"),
          Map.entry(20, "Swerve/FR/Drive"),
          Map.entry(21, "Swerve/FR/Angle"),
          Map.entry(30, "Swerve/BR/Drive"),
          Map.entry(31, "Swerve/BR/Angle"),
          Map.entry(40, "Swerve/BL/Drive"),
          Map.entry(41, "Swerve/BL/Angle")
      );

    // Swerve CAN IDs
    public static final class SwerveModules {
        // FL
        public static final SwerveModuleCanIDs MOD0 = 
            new SwerveModuleCanIDs(10, 11);

        // FR
        public static final SwerveModuleCanIDs MOD1 = 
            new SwerveModuleCanIDs(20, 21);

        // BL
        public static final SwerveModuleCanIDs MOD2 = 
            new SwerveModuleCanIDs(40, 41);

        // BR
        public static final SwerveModuleCanIDs MOD3 = 
            new SwerveModuleCanIDs(30, 31);
    }

    // TODO Put constants for your other CAN devices here




    /**
     * Utility class to hold all the CAN IDs for a swerve module.
     */
    public static final class SwerveModuleCanIDs
    {
        public final int DRIVE_MOTOR_ID;
        public final int ANGLE_MOTOR_ID;

        public SwerveModuleCanIDs(int driveMotorID, int angleMotorID)
        {
            this.ANGLE_MOTOR_ID = angleMotorID;
            this.DRIVE_MOTOR_ID = driveMotorID;
        }
    }
    
}
