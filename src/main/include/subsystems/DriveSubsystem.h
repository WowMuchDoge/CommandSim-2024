#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/AnalogGyro.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/system/plant/LinearSystemId.h>

class DriveSubsystem : public frc2::SubsystemBase {
    public:
        DriveSubsystem(double controller) : controller{controller} {
            m_leftEncoder.SetDistancePerPulse(2 * 3.14159265358979323846 * 6 / 100 );
            m_rightEncoder.SetDistancePerPulse(2 * 3.14159265358979323846 * 6 / 100);

            frc::SmartDashboard::PutData("Field", &m_field);
        };

        void SimulationPeriodic() override;

        void Periodic() override;

        void ArcadeDrive(double fwd, double rot);

    private:
        frc::PWMSparkMax m_leftLeadMotor{0};
        frc::PWMSparkMax m_rightLeadMotor{1};
        frc::PWMSparkMax m_leftFollowMotor{2};
        frc::PWMSparkMax m_rightFollowMotor{3};

        frc::Encoder m_leftEncoder{0, 1};
        frc::Encoder m_rightEncoder{2, 3};

        frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
        frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};

        frc::AnalogGyro m_gyro{1};

        frc::sim::AnalogGyroSim m_gyroSim{m_gyro};

        frc::sim::DifferentialDrivetrainSim m_driveSim{
            frc::DCMotor::NEO(2),
            10.86,
            2.1_kg_sq_m,
            26.5_kg,
            6_in,
            0.546_m,
        };

        frc::DifferentialDriveOdometry m_odometry{
            m_gyro.GetRotation2d(),
            units::meter_t{m_leftEncoder.GetDistance()},
            units::meter_t{m_rightEncoder.GetDistance()},
        };

        frc::Field2d m_field;

        frc::DifferentialDrive m_drive{m_leftLeadMotor, m_rightLeadMotor};

        frc::LinearSystem<2, 2, 2> m_drivetrainSystem =
            frc::LinearSystemId::IdentifyDrivetrainSystem(
                1.98_V / 1_mps, 0.2_V / 1_mps_sq, 1.5_V / 1_mps, 0.3_V / 1_mps_sq);
        frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
        m_drivetrainSystem, 1_in, frc::DCMotor::CIM(2), 8, 2_in};

        double controller;
};