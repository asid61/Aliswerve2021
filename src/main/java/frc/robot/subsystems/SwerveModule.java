// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    public PWMVictorSPX drive;
    public PWMVictorSPX rot;

    private static final double ROT_P = 1.1;
    private static final double ROT_I = 0;
    private static final double ROT_D = 11;

    private boolean ROTATION_SENSOR_PHASE;
    private boolean ROTATION_INVERT;

    public PIDController drivePID;
    public PIDController rotPID;
    

    public SwerveModule(int rotPort, int drivePort,
            boolean rotationSensorPhase, boolean rotationInverted) {

        ROTATION_SENSOR_PHASE = rotationSensorPhase;
        ROTATION_INVERT = rotationInverted;

        drive = new PWMVictorSPX(drivePort);
        rot = new PWMVictorSPX(rotPort);

    }

    // set both motors for the module
    public void setMotors(double drivespeed, double rotspeed) {
        drive.set(drivespeed);
        rot.set(rotspeed);
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
