#pragma once 

#include <frc2/command/Command.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "../subsystems/DriveSubsystem.h"

class DefaultDrive : frc2::CommandHelper<frc2::Command, DefaultDrive> {
    public:
        DefaultDrive(DriveSubsystem *subsystem, std::function<double()> forward, 
                     std::function<double()> rotation)
        : m_drive{subsystem},
          m_forward{forward},
          m_rotation{rotation} {
            AddRequirements({subsystem});
          };

    void Execute() override {
        double joystickY = m_rotation();
        double joystickX = m_forward();

        m_drive->ArcadeDrive(joystickY, joystickX);
    }

    private:
        DriveSubsystem* m_drive;
        std::function<double()> m_forward;
        std::function<double()> m_rotation;
};