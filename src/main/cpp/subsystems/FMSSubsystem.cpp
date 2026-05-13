#include <subsystems/FMSSubsystem.h>
#include <bearlog/bearlog.h>
#include <frc/smartdashboard/SmartDashboard.h>

FMSSubsystem::FMSSubsystem() {
    m_manualShift = false;
}

void FMSSubsystem::Periodic() {
    BearLog::Log("FMS/MatchTime", GetMatchTime());
    CurrentShift();
    BearLog::Log("FMS/MyAllianceShift?", MyAllianceShift());
}

FMSSubsystem::autoWinner FMSSubsystem::AutoWinner() {
    std::string gameData = frc::DriverStation::GetGameSpecificMessage();
    if (!gameData.empty()) {
        if (gameData[0] == 'R') {
            return FMSSubsystem::autoWinner::kRed;
        } else if (gameData[0] == 'B') {
            return FMSSubsystem::autoWinner::kBlue;
        } else {
            return FMSSubsystem::autoWinner::kInvalidInfo;
        }
    } else {
        return FMSSubsystem::autoWinner::kNoInfo;
    }
}

units::second_t FMSSubsystem::GetMatchTime() {
    return units::second_t(std::round(frc::DriverStation::GetMatchTime().value()));
}

frc2::CommandPtr FMSSubsystem::ManualShift(std::string roboalliance) {
    return frc2::cmd::Run([this, roboalliance] {
        if (roboalliance == "Red") {
            m_autoWinner = allianceShift::kRedShift;
            m_autoLoser = allianceShift::kBlueShift;
        } else {
            m_autoWinner = allianceShift::kBlueShift;
            m_autoLoser = allianceShift::kRedShift;
        }
        m_manualShift = true;
    });
}

enum FMSSubsystem::allianceShift FMSSubsystem::CurrentShift() {
    if (GetMatchTime() > 130_s){
        BearLog::Log("Next Shift", GetMatchTime() - 130_s);
        BearLog::Log("Alliance Shift", std::string("#ffffffff"));
        return allianceShift::kBothShift;
    } else if ((GetMatchTime() <= 130_s) && (GetMatchTime() > 30_s)) {
        BearLog::Log("ManualShiftActivated?", m_manualShift);
        if (m_manualShift == false) {
            if (AutoWinner() == FMSSubsystem::autoWinner::kRed) {
                m_autoWinner = allianceShift::kRedShift;
                m_autoLoser = allianceShift::kBlueShift;
            } else if (AutoWinner() == FMSSubsystem::autoWinner::kBlue) {
                m_autoWinner = allianceShift::kBlueShift;
                m_autoLoser = allianceShift::kRedShift;
            } else {
                m_autoWinner = allianceShift::kNoShift;
                m_autoLoser = allianceShift::kNoShift;
            }
        }

        int currentShift;
        if ((GetMatchTime() <= 130_s) && (GetMatchTime() > 105_s)) {
            BearLog::Log("Next Shift", GetMatchTime() - 105_s);
            currentShift = 4;
        } else if ((GetMatchTime() <= 105_s) && (GetMatchTime() > 80_s)) {
            BearLog::Log("Next Shift", GetMatchTime() - 80_s);
            currentShift = 3;
        } else if ((GetMatchTime() <= 80_s) && (GetMatchTime() > 55_s)) {
            BearLog::Log("Next Shift", GetMatchTime() - 55_s);
            currentShift = 2;
        } else if ((GetMatchTime() <= 55_s) && (GetMatchTime() >= 30_s)) {
            BearLog::Log("Next Shift", GetMatchTime() - 30_s);
            currentShift = 1;
        } else {
            currentShift = 0;
        }

        if (!(currentShift == 0)) {
            if (currentShift % 2 == 0) {
                if (m_autoLoser == allianceShift::kRedShift) {
                    BearLog::Log("Alliance Shift", std::string("#FF0000"));
                } else {
                    BearLog::Log("Alliance Shift", std::string("#0000FF"));
                }
                return m_autoLoser;
            } else {
                if (m_autoWinner == allianceShift::kRedShift) {
                    BearLog::Log("Alliance Shift", std::string("#FF0000"));
                } else {
                    BearLog::Log("Alliance Shift", std::string("#0000FF"));
                }
                return m_autoWinner;
            }
        }
    } else if ((GetMatchTime() <= 30_s) && (GetMatchTime() >= 0_s)) {
        BearLog::Log("Alliance Shift", std::string("#ffffffff"));
        BearLog::Log("Next Shift", GetMatchTime());
        return allianceShift::kBothShift;
    } else {
        BearLog::Log("Alliance Shift", std::string("#000000ff"));
        return allianceShift::kNoShift;
    }
}

bool FMSSubsystem::MyAllianceShift() {
    using namespace frc;
    allianceShift myAlliance;
    if (DriverStation::GetAlliance() == DriverStation::Alliance::kRed) {
        myAlliance = allianceShift::kRedShift;
    } else if (DriverStation::GetAlliance() == DriverStation::Alliance::kBlue) {
        myAlliance = allianceShift::kBlueShift;
    } else {
        myAlliance = allianceShift::kNoShift;
    }

    if (CurrentShift() == allianceShift::kBothShift) {
        return true;
    } else {
        return (myAlliance == CurrentShift());
    }
}