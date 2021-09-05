// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveModule extends SubsystemBase {
    public PWMVictorSPX drive;
    public PWMVictorSPX rot;

    private static final double ROT_KP = 1.8;
    private static final double ROT_KI = 0.5;
    private static final double ROT_KD = 0.01;

    private double DRIVE_KF = 0.3; //0.2;
    private double DRIVE_KS = 0.04;

    private PIDController rotPIDController = new PIDController(ROT_KP, ROT_KI, ROT_KD);

    //private AnalogPotentiometer rotPot;
    private AnalogInput rotPot;

    private double ROT_OFFSET;
    private boolean ROT_PHASE;

    public SwerveModule(int rotPort, int drivePort, int rotPotPort, boolean driveinverted, boolean rotationInverted,
            boolean potInverted) {

        drive = new PWMVictorSPX(drivePort);
        rot = new PWMVictorSPX(rotPort);
        //rotPot = new AnalogPotentiometer(rotPotPort, 2.0*Math.PI, -Math.PI);
        rotPot = new AnalogInput(rotPotPort);


        ROT_PHASE = potInverted;

        drive.setInverted(driveinverted);
        rot.setInverted(rotationInverted);
        rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
        rotPIDController.setIntegratorRange(-0.1, 0.1);

    }

    // set both motors for the module
    public void setMotors(double drivespeed, double rotspeed) {
        drive.set(drivespeed);
        rot.set(rotspeed);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        double potangle = getAngle();
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(potangle));
        //SwerveModuleState state = desiredState;

        // Calculate the drive output from the drive PID controller.
        double driveOutput = state.speedMetersPerSecond * DRIVE_KF + Math.signum(state.speedMetersPerSecond) * DRIVE_KS;

        // Calculate the turning motor output from the turning PID controller.
        double rotOutput = rotPIDController.calculate(potangle, state.angle.getRadians());

        // Set motor outputs.
        drive.set(driveOutput);
        rot.set(rotOutput);
    }

    public void setRotationOffset(double rotoffset) {
        ROT_OFFSET = rotoffset;
    }

    public double getAngle() {
        double potValue = rotPot.getValue()/2590.0 * 2 * Math.PI - Math.PI;
        potValue = potValue - ROT_OFFSET;
        while(potValue <= -Math.PI) {potValue = potValue + 2*Math.PI;}
        while(potValue >= Math.PI) {potValue = potValue - 2*Math.PI;}

        double potangle = ROT_PHASE ? potValue : -potValue;
        return potangle;
    }

    public void setDrivePID(double newkF, double newKS) {
        DRIVE_KF = newkF;
        DRIVE_KS = newKS;
    }

    public PIDController getPIDController() {
        return rotPIDController;
    }

    public AnalogInput getPot() {
        return rotPot;
    }
    // get rotation speed controller
    public PWMVictorSPX getRotMotor() {
        return rot;
    }

    // get drive speedcontroller
    public PWMVictorSPX getDriveMotor() {
        return drive;
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
