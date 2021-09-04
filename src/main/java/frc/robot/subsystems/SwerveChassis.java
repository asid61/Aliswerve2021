// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
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
    private static final boolean RT_ANG_PHASE = false;
    private static final boolean RT_ANG_INVERT = false;
    private static final boolean LT_ANG_PHASE = false;
    private static final boolean LT_ANG_INVERT = false;

    SwerveDriveKinematics kinematics;

    public SwerveChassis() {
        Translation2d frontLocation = new Translation2d(0, 0.337);
        Translation2d backLeftLocation = new Translation2d(0.292, -0.169);
        Translation2d backRightLocation = new Translation2d(-0.292, -0.169);

        FT = new SwerveModule(FT_ANG_PORT, FT_DRIVE_PORT, FT_ANG_PHASE, FT_ANG_INVERT);
        RT = new SwerveModule(RT_ANG_PORT, RT_DRIVE_PORT, RT_ANG_PHASE, RT_ANG_INVERT);
        LT = new SwerveModule(LT_ANG_PORT, LT_DRIVE_PORT, LT_ANG_PHASE, LT_ANG_INVERT);

        kinematics = new SwerveDriveKinematics(frontLocation, backLeftLocation, backRightLocation);

    }

    public void SetAllMotors() {
        FT.setMotors(0.1, 0.5);
        RT.setMotors(0.1, 0.5);
        LT.setMotors(0.1, 0.5);
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