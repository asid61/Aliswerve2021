// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveChassis extends SubsystemBase {

    private SwerveModule FT;
    private SwerveModule RT;
    private SwerveModule LT;

    // ports for each motor
    private static final int FT_DRIVE_PORT = 0;
    private static final int FT_ANG_PORT = 1;
    private static final int RT_DRIVE_PORT = 2;
    private static final int RT_ANG_PORT = 3;
    private static final int LT_DRIVE_PORT = 4;
    private static final int LT_ANG_PORT = 5;

    // ports for angular feedback
    private static final int FT_POT_PORT = 0;
    private static final int RT_POT_PORT = 1;
    private static final int LT_POT_PORT = 2;

    // rotation offsets per module
    private static final int FT_OFFSET = 1000; // FT = front
    private static final int RT_OFFSET = 1000; // RT = right
    private static final int LT_OFFSET = 1000; // LT = left

    private static final boolean FT_ANG_PHASE = false;
    private static final boolean FT_ANG_INVERT = false;
    private static final boolean FT_DRIVE_INVERT = false;
    private static final boolean RT_ANG_PHASE = false;
    private static final boolean RT_ANG_INVERT = false;
    private static final boolean RT_DRIVE_INVERT = false;
    private static final boolean LT_ANG_PHASE = false;
    private static final boolean LT_ANG_INVERT = false;
    private static final boolean LT_DRIVE_INVERT = false;

    // kinematics constants
    SwerveDriveKinematics kinematics;
    private static final double MAX_SPEED = 4.0; // in m/s 

    public SwerveChassis() {
        Translation2d frontLocation = new Translation2d(0, 0.337);
        Translation2d backLeftLocation = new Translation2d(0.292, -0.169);
        Translation2d backRightLocation = new Translation2d(-0.292, -0.169);

        FT = new SwerveModule(FT_ANG_PORT, FT_DRIVE_PORT, FT_POT_PORT, FT_DRIVE_INVERT, FT_ANG_INVERT, FT_ANG_PHASE);

        RT = new SwerveModule(RT_ANG_PORT, RT_DRIVE_PORT, RT_POT_PORT, RT_DRIVE_INVERT, RT_ANG_INVERT, RT_ANG_PHASE);

        LT = new SwerveModule(LT_ANG_PORT, LT_DRIVE_PORT, LT_POT_PORT, LT_DRIVE_INVERT, LT_ANG_INVERT, LT_ANG_PHASE);

        FT.setRotationOffset(FT_OFFSET);
        RT.setRotationOffset(RT_OFFSET);
        LT.setRotationOffset(LT_OFFSET);

        kinematics = new SwerveDriveKinematics(frontLocation, backLeftLocation, backRightLocation);

    }

    public void SetAllMotors() {
        FT.setMotors(0.1, 0.5);
        RT.setMotors(0.1, 0.5);
        LT.setMotors(0.1, 0.5);
    }

    public void Drive(double xSpeed, double ySpeed, double rot) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, MAX_SPEED);
        FT.setDesiredState(swerveModuleStates[0]);
        RT.setDesiredState(swerveModuleStates[1]);
        LT.setDesiredState(swerveModuleStates[2]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}