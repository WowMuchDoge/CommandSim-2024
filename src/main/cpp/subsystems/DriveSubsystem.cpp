#include <frc2/command/CommandPtr.h>
#include <frc2/command/Subsystem.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/RobotController.h>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "../../include/subsystems/DriveSubsystem.h"

void DriveSubsystem::Periodic() {
    m_odometry.Update(m_gyro.GetRotation2d(), units::meter_t(m_leftEncoder.GetDistance()), 
                                              units::meter_t(m_rightEncoder.GetDistance()));
    m_field.SetRobotPose(m_odometry.GetPose());

    frc::SmartDashboard::PutNumber("Controller: ", controller);
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