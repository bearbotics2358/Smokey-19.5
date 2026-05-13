#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>


class FMSSubsystem : public frc2::SubsystemBase {
public:
    FMSSubsystem();
    
    units::second_t GetMatchTime();

    void Periodic() override;
    
    enum class allianceShift{
        kNoShift,
        kBlueShift,
        kRedShift,
        kBothShift
    };

    enum class autoWinner{
        kRed,
        kBlue,
        kInvalidInfo,
        kNoInfo
    };

    autoWinner AutoWinner();
    allianceShift CurrentShift();
    
    allianceShift m_autoWinner;
    allianceShift m_autoLoser;

    frc2::CommandPtr ManualShift(std::string alliance);
    bool m_manualShift;

    bool MyAllianceShift();
};