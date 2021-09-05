// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;


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

    // rotation offsets per module in radians
    private static final double FT_OFFSET = 0.1; // FT = front
    private static final double RT_OFFSET = 0.9; // RT = right
    private static final double LT_OFFSET = -1.1; // LT = left

    private static final boolean FT_ANG_PHASE = false;
    private static final boolean FT_ANG_INVERT = true;
    private static final boolean FT_DRIVE_INVERT = true;
    private static final boolean RT_ANG_PHASE = false;
    private static final boolean RT_ANG_INVERT = true;
    private static final boolean RT_DRIVE_INVERT = true;
    private static final boolean LT_ANG_PHASE = false;
    private static final boolean LT_ANG_INVERT = true;
    private static final boolean LT_DRIVE_INVERT = true;

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

        SmartDashboard.putNumber("Foffs", FT_OFFSET);
        SmartDashboard.putNumber("Roffs", RT_OFFSET);
        SmartDashboard.putNumber("Loffs", LT_OFFSET);
        SmartDashboard.putNumber("kp", FT.getPIDController().getP());
        SmartDashboard.putNumber("ki", FT.getPIDController().getI());
        SmartDashboard.putNumber("kd", FT.getPIDController().getD());

        kinematics = new SwerveDriveKinematics(frontLocation, backLeftLocation, backRightLocation);

    }

    public void SetAllMotors(double value) {
        FT.setMotors(value, 0.5);
        RT.setMotors(value, 0.5);
        LT.setMotors(value, 0.5);
    }

    public void Drive(double xSpeed, double ySpeed, double rot) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, MAX_SPEED);
        FT.setDesiredState(swerveModuleStates[0]);
        RT.setDesiredState(swerveModuleStates[1]);
        LT.setDesiredState(swerveModuleStates[2]);

        SmartDashboard.putNumber("targetang", swerveModuleStates[0].angle.getRadians());
        SmartDashboard.putNumber("target", swerveModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("PIDtarget", FT.getPIDController().getSetpoint());
        SmartDashboard.putNumber("PIDerror", FT.getPIDController().getPositionError());

        double newoffsetF = SmartDashboard.getNumber("Foffs", FT_OFFSET);
        double newoffsetR = SmartDashboard.getNumber("Roffs", RT_OFFSET);
        double newoffsetL = SmartDashboard.getNumber("Loffs", LT_OFFSET);
        double newp = SmartDashboard.getNumber("kp", FT.getPIDController().getP());
        double newi = SmartDashboard.getNumber("ki", FT.getPIDController().getI());
        double newd = SmartDashboard.getNumber("kd", FT.getPIDController().getD());
        FT.setRotationOffset(newoffsetF);
        FT.getPIDController().setP(newp);
        FT.getPIDController().setI(newi);
        FT.getPIDController().setD(newd);

        RT.setRotationOffset(newoffsetR);
        RT.getPIDController().setP(newp);
        RT.getPIDController().setI(newi);
        RT.getPIDController().setD(newd);

        LT.setRotationOffset(newoffsetL);
        LT.getPIDController().setP(newp);
        LT.getPIDController().setI(newi);
        LT.getPIDController().setD(newd);
        
    }

    public SwerveModule getFT() {
        return FT;
    }

    public SwerveModule getRT() {
        return RT;
    }

    public SwerveModule getLT() {
        return LT;
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