#include <frc2/command/CommandPtr.h>
#include <frc2/command/Subsystem.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/RobotController.h>
#include <math.h>

#include "../../include/subsystems/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem() {
    m_leftEncoder.SetDistancePerPulse(2 * 3.14159265358979323846 * 6 / 100 );
    m_rightEncoder.SetDistancePerPulse(2 * 3.14159265358979323846 * 6 / 100);

    frc::SmartDashboard::PutData("Field", &m_field);
}

void DriveSubsystem::Periodic() {
    m_odometry.Update(m_gyro.GetRotation2d(), units::meter_t(m_leftEncoder.GetDistance()), 
                                              units::meter_t(m_rightEncoder.GetDistance()));
    m_field.SetRobotPose(m_odometry.GetPose());
}

void DriveSubsystem::SimulationPeriodic() {
    m_driveSim.SetInputs(
        m_leftLeadMotor.Get() * units::volt_t(frc::RobotController::GetInputVoltage()),
        m_rightLeadMotor.Get() * units::volt_t(frc::RobotController::GetInputVoltage()));

        m_driveSim.Update(20_ms);

        m_leftEncoderSim.SetDistance(m_driveSim.GetLeftPosition().value());
        m_leftEncoderSim.SetRate(m_driveSim.GetLeftPosition().value());
        m_rightEncoderSim.SetDistance(m_driveSim.GetLeftVelocity().value());
        m_rightEncoderSim.SetRate(m_driveSim.GetRightPosition().value());
        m_gyroSim.SetAngle(-m_driveSim.GetHeading().Degrees().value());
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
    m_drive.ArcadeDrive(fwd, rot);
}