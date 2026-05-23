#pragma once

#include <mutex>

#include <frc/geometry/Pose3d.h>
#include <frc/Notifier.h>
#include <frc/PowerDistribution.h>
#include <frc/RobotController.h>

#include "bearlog/internal/data_log_writer.h"
#include "bearlog/internal/network_tables_writer.h"

class BearLogOptions {
public:
  enum class NTPublish {No, Yes};
  enum class LogWithNTPrefix {No, Yes};
  enum class LogExtras {No, Yes};

  /**
   * Use enum classes as parameters instead of bools:
   * - Better type safety
   * - Allow options to be expanded more easily in the future
   * - Be more clear about which option is being set
   */
  BearLogOptions(NTPublish ntPublish = NTPublish::Yes,
                 LogWithNTPrefix withNTPrefix = LogWithNTPrefix::Yes,
                 LogExtras logExtras = LogExtras::No)
      : m_NtPublish(ntPublish),
        m_LogWithNTPrefix(withNTPrefix),
        m_LogExtras(logExtras) {}

  bool ShouldPublishToNetworkTables() {
    return m_NtPublish == NTPublish::Yes;
  }

  bool ShouldLogToFileWithNTPrefix() {
    return m_LogWithNTPrefix == LogWithNTPrefix::Yes;
  }

  bool ShouldLogExtras() {
    return m_LogExtras == LogExtras::Yes;
  }

private:
  NTPublish m_NtPublish;
  LogWithNTPrefix m_LogWithNTPrefix;
  LogExtras m_LogExtras;
};

class BearLog {
  const std::string kLogTable = "/Robot";

public:
  // Delete the copy constructor. BearLog should not be cloneable.
  BearLog(const BearLog&) = delete;

  ~BearLog() {
    m_InternalLogNotifier.Stop();
  }

  // Preventing assigning a BearLog object
  BearLog& operator=(const BearLog&) = delete;

  static void SetOptions(BearLogOptions options) {
    GetInstance().m_Options = options;

    GetInstance().m_DataLogger.SetShouldUseNTTablePrefix(options.ShouldLogToFileWithNTPrefix());

    StartLoggingExtrasIfNeeded();
  }

  static void StartLoggingExtrasIfNeeded() {
    if (GetInstance().m_Options.ShouldLogExtras()) {
      // Enable the logging loop to log the PDH information (and other extras) at 50Hz
      static const units::second_t kLoopPeriodSeconds = 0.02_s;
      GetInstance().m_InternalLogNotifier.StartPeriodic(kLoopPeriodSeconds);
    }
  }

  void LogExtras() {
    if (IsEnabled()) {
      LogPdh();
    }
  }

  void LogPdh() {
    // Use std::lock_guard to lock access to m_Pdh for the scope of this function. This prevents other functions
    // from modifying it.
    const std::lock_guard<std::mutex> lock(GetInstance().m_PdhMutex);

    if (!GetInstance().m_Pdh) {
      return;
    }

    static const std::string kPdhPrefix = "SystemStats/PowerDistribution/";

    BearLog::Log(kPdhPrefix + "Temperature(C)", GetInstance().m_Pdh->GetTemperature());
    BearLog::Log(kPdhPrefix + "Voltage(V)", GetInstance().m_Pdh->GetVoltage());
    BearLog::Log(kPdhPrefix + "ChannelCurrent(A)", GetInstance().m_Pdh->GetAllCurrents());
    BearLog::Log(kPdhPrefix + "TotalCurrent(A)", GetInstance().m_Pdh->GetTotalCurrent());
    BearLog::Log(kPdhPrefix + "TotalPower(W)", GetInstance().m_Pdh->GetTotalPower());
    BearLog::Log(kPdhPrefix + "TotalEnergy(J)", GetInstance().m_Pdh->GetTotalEnergy());
    BearLog::Log(kPdhPrefix + "ChannelCount", GetInstance().m_Pdh->GetNumChannels());
  }

  static void SetPdh(std::shared_ptr<frc::PowerDistribution> powerDistribution) {
    // Use std::lock_guard to lock access to m_Pdh for the scope of this function. This prevents other functions
    // from trying to access it while this function is modifying it.
    const std::lock_guard<std::mutex> lock(GetInstance().m_PdhMutex);

    GetInstance().m_Pdh = powerDistribution;
  }

  static void DeletePdh() {
    // Use std::lock_guard to lock access to m_Pdh for the scope of this function. This prevents other functions
    // from trying to access it while this function is modifying it.
    const std::lock_guard<std::mutex> lock(GetInstance().m_PdhMutex);

    GetInstance().m_Pdh = nullptr;
  }

  static void SetEnabled(bool newEnabled) {
    GetInstance().m_IsEnabled = newEnabled;

    if (newEnabled) {
      // Start logging extras if that option is selected and BearLog is being enabled
      StartLoggingExtrasIfNeeded();
    } else {
      // When disabling logging, be sure to stop the internal notifier for the extras
      GetInstance().m_InternalLogNotifier.Stop();
    }
  }

  static bool IsEnabled() {
    return GetInstance().m_IsEnabled;
  }

  static void Log(std::string key, bool value) {
    if (!IsEnabled()) {
      return;
    }

    uint64_t now = frc::RobotController::GetFPGATime();

    GetInstance().m_DataLogger.Log(now, key, value);
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key, value);
    }
  }

  static void Log(std::string key, const std::vector<double>& value) {
    if (!IsEnabled()) {
      return;
    }

    uint64_t now = frc::RobotController::GetFPGATime();

    GetInstance().m_DataLogger.Log(now, key, value);
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key, value);
    }
  }

  static void Log(std::string key, double value) {
    if (!IsEnabled()) {
      return;
    }

    uint64_t now = frc::RobotController::GetFPGATime();

    GetInstance().m_DataLogger.Log(now, key, value);
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key, value);
    }
  }

  static void Log(std::string key, int value) {
    if (!IsEnabled()) {
      return;
    }

    uint64_t now = frc::RobotController::GetFPGATime();

    GetInstance().m_DataLogger.Log(now, key, value);
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key, value);
    }
  }

  static void Log(std::string key, std::span<const std::string> value) {
    if (!IsEnabled()) {
      return;
    }

    uint64_t now = frc::RobotController::GetFPGATime();

    GetInstance().m_DataLogger.Log(now, key, value);
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key, value);
    }
  }

  static void Log(std::string key, const std::string& value) {
    if (!IsEnabled()) {
      return;
    }

    uint64_t now = frc::RobotController::GetFPGATime();

    GetInstance().m_DataLogger.Log(now, key, value);
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key, value);
    }
  }

  static void Log(std::string key, const frc::Pose3d& value) {
    if (!IsEnabled()) {
      return;
    }

    uint64_t now = frc::RobotController::GetFPGATime();

    GetInstance().m_DataLogger.Log(now, key, value);
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key, value);
    }
  }

  static void Log(std::string key, std::vector<frc::Pose3d>& value) {
    if (!IsEnabled()) {
      return;
    }

    uint64_t now = frc::RobotController::GetFPGATime();

    GetInstance().m_DataLogger.Log(now, key, value);
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key, value);
    }
  }

  template<typename Units>
  static void Log(std::string key, Units value) {
    if (!IsEnabled()) {
      return;
    }

    // @todo Add a better compiler error message when the Units class doesn't support the necessary functions

    uint64_t now = frc::RobotController::GetFPGATime();

    std::string key_with_units = key + "(" + value.abbreviation() + ")";

    GetInstance().m_DataLogger.Log(now, key_with_units, value.value());
    if (GetInstance().m_Options.ShouldPublishToNetworkTables()) {
      GetInstance().m_NTLogger.Log(now, key_with_units, value.value());
    }
  }

private:
  // Make the constructor private to disallow direct instantiation of BearLog
  BearLog()
      : m_IsEnabled(true),
        m_DataLogger(kLogTable),
        m_NTLogger(kLogTable),
        m_InternalLogNotifier(&BearLog::LogExtras, this) {
  }

  static BearLog& GetInstance() {
    static BearLog instance;
    return instance;
  }

  // Mutex to protect multiple threads accessing m_Pdh
  std::mutex m_PdhMutex;

  bool m_IsEnabled;
  DataLogWriter m_DataLogger;
  NetworkTablesWriter m_NTLogger;
  BearLogOptions m_Options;
  std::shared_ptr<frc::PowerDistribution> m_Pdh;
  frc::Notifier m_InternalLogNotifier;
};