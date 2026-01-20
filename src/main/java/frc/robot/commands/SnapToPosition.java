// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Swerve.*;


public class SnapToPosition extends Command 
{
    public final ArrayList<Pose2d> snapPositions= new ArrayList<Pose2d>(){{
        // TODO Add field coordinates you want the robot to snap to. For example:
        add(new Pose2d(3.261, 4.403, new Rotation2d(Radians.convertFrom(-90, Degrees))));//blue a
        add(new Pose2d(3.261, 3.917, new Rotation2d(Radians.convertFrom(-90, Degrees))));//blue b
        
    }};
    
    public Swerve swerve;
    private Pose2d target;
    public PIDController x;
    public PIDController y;
    public PIDController r;

    public SnapToPosition(Swerve s){

        swerve = s;
        x = new PIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD);
        y = new PIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD);
        r = new PIDController(ROTATION_PID.kP, ROTATION_PID.kI, ROTATION_PID.kD);
        r.enableContinuousInput(180,-180);
        addRequirements(swerve);
        
    }
    // Called when the command is initially scheduled
    @Override
    public void initialize(){
        Pose2d currentpose = swerve.getPose();
        // target = currentpose.nearest(snapPositions);

        double bestDistance = Double.MAX_VALUE;

        for(Pose2d p : snapPositions){
            double distance = currentpose.getTranslation().getDistance(p.getTranslation());
            if (distance < bestDistance){
                target = p;
                bestDistance = distance;
            }
        }

        swerve.field.getObject("SnapToPose Target").setPose(target);
        
        x.setSetpoint(target.getX());
        y.setSetpoint(target.getY());
        r.setSetpoint(target.getRotation().getDegrees());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
       swerve.drive(new Translation2d(-getXPower(),-getYPower()), getRPower(), true, false, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return false;
    }

    public double getXPower(){
        return x.calculate(swerve.getPose().getX());
    }

    public double getYPower(){
        return y.calculate(swerve.getPose().getY());
    }

    public double getRPower(){
        return r.calculate(swerve.getPose().getRotation().getDegrees());
    }

}