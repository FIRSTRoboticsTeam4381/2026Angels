// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

/** Constants file for CAN IDs */
public final class CanIDs {

    
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
    //public static final int EXAMPLE_MOTOR_ID = 45;
    public static final int SHOOTER_1_MOTOR_ID = 45;
    public static final int SHOOTER_2_MOTOR_ID = 46;
    public static final int INTAKER_MOTOR_ID = 47;
    public static final int AGITATOR_MOTOR_ID = 48;
    public static final int FUNNEL_MOTOR_ID = 49;
    public static final int HOODED_MOTOR_MOTOR_ID = 50;
    public static final int INTAKE_PIVOT_MOTOR_ID = 51;
    public static final int HOPPER_SLIDE_MOTOR_ID = 52;
    /**
     * Utility class to hold all the CAN IDs for a swerve module.
     */
    public static final class SwerveModuleCanIDs
    {
        public final int driveMotorID;
        public final int angleMotorID;

        public SwerveModuleCanIDs(int driveMotorID, int angleMotorID)
        {
            this.angleMotorID = angleMotorID;
            this.driveMotorID = driveMotorID;
        }
    }
    

    // Map of CAN IDs for URCL
    // TODO Keep this up to date with the actual CAN IDs so logs make sense!
    public static final Map<Integer, String> URCL_IDS = Map.ofEntries(
          Map.entry(10, "Swerve/FL/Drive"),
          Map.entry(11, "Swerve/FL/Angle"),
          Map.entry(20, "Swerve/FR/Drive"),
          Map.entry(21, "Swerve/FR/Angle"),
          Map.entry(30, "Swerve/BR/Drive"),
          Map.entry(31, "Swerve/BR/Angle"),
          Map.entry(40, "Swerve/BL/Drive"),
          Map.entry(41, "Swerve/BL/Angle"),
          Map.entry(SHOOTER_1_MOTOR_ID, "Shooter/motor1"),
          Map.entry(SHOOTER_2_MOTOR_ID, "Shooter/motor2"),
          Map.entry(INTAKER_MOTOR_ID, "Intaker/motor1"),
          Map.entry(AGITATOR_MOTOR_ID, "Agitator/Agitator/motor1"),
          Map.entry(FUNNEL_MOTOR_ID, "Agitator/Funnel/motor1"),
          Map.entry(HOODED_MOTOR_MOTOR_ID, "CarHood/motor1"),
          Map.entry(INTAKE_PIVOT_MOTOR_ID, "IntakePivot/IntakePivot/motor1"),
          Map.entry(HOPPER_SLIDE_MOTOR_ID, "IntakePivot/Hopperslide/motor1")
          //Map.entry(EXAMPLE_MOTOR_ID, "Example/motor1")
      );

}
