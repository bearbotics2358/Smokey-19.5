#pragma once

#include <string>
#include <unordered_map>
#include "frc/DataLogManager.h"
#include "frc/geometry/Pose3d.h"
#include "wpi/DataLog.h"

class DataLogWriter {
public:
  DataLogWriter(const std::string& logTable):
        m_LogTable(logTable), m_Log(frc::DataLogManager::GetLog()) {
  }

  void Log(uint64_t timestamp, std::string key, bool value) {
    if (m_BooleanLogs.contains(key)) {
      m_BooleanLogs.at(key).Append(value);
    } else {
      auto new_boolean_log = wpi::log::BooleanLogEntry(m_Log, GetPrefixKey(key), kEntryMetadata, timestamp);

      new_boolean_log.Append(value);
      m_BooleanLogs[key] = std::move(new_boolean_log);
    }
  }

  void Log(uint64_t timestamp, std::string key, std::span<const double> value) {
    if (m_DoubleArrayLogs.contains(key)) {
      m_DoubleArrayLogs.at(key).Append(value);
    } else {
      auto new_double_array_log = wpi::log::DoubleArrayLogEntry(m_Log, GetPrefixKey(key), kEntryMetadata, timestamp);

      new_double_array_log.Append(value);
      m_DoubleArrayLogs[key] = std::move(new_double_array_log);
    }
  }

  void Log(uint64_t timestamp, std::string key, double value) {
    if (m_DoubleLogs.contains(key)) {
      m_DoubleLogs.at(key).Append(value);
    } else {
      auto new_double_log = wpi::log::DoubleLogEntry(m_Log, GetPrefixKey(key), kEntryMetadata, timestamp);

      new_double_log.Append(value);
      m_DoubleLogs[key] = std::move(new_double_log);
    }
  }

  void Log(uint64_t timestamp, std::string key, int value) {
    if (m_IntegerLogs.contains(key)) {
      m_IntegerLogs.at(key).Append(value);
    } else {
      auto new_integer_log = wpi::log::IntegerLogEntry(m_Log, GetPrefixKey(key), kEntryMetadata, timestamp);

      new_integer_log.Append(value);
      m_IntegerLogs[key] = std::move(new_integer_log);
    }
  }

  void Log(uint64_t timestamp, std::string key, std::span<const std::string> value) {
    if (m_StringArrayLogs.contains(key)) {
      m_StringArrayLogs.at(key).Append(value);
    } else {
      auto new_string_array_log = wpi::log::StringArrayLogEntry(m_Log, GetPrefixKey(key), kEntryMetadata, timestamp);

      new_string_array_log.Append(value);
      m_StringArrayLogs[key] = std::move(new_string_array_log);
    }
  }

  void Log(uint64_t timestamp, std::string key, const std::string& value) {
    if (m_StringLogs.contains(key)) {
      m_StringLogs.at(key).Append(value);
    } else {
      auto new_string_log = wpi::log::StringLogEntry(m_Log, GetPrefixKey(key), kEntryMetadata, timestamp);

      new_string_log.Append(value);
      m_StringLogs[key] = std::move(new_string_log);
    }
  }

  void Log(uint64_t timestamp, std::string key, const frc::Pose3d& value) {
    if (m_PoseLogs.contains(key)) {
      m_PoseLogs.at(key).Append(value);
    } else {
      auto new_pose_log = wpi::log::StructLogEntry<frc::Pose3d>(m_Log, GetPrefixKey(key), kEntryMetadata, timestamp);

      new_pose_log.Append(value);
      m_PoseLogs[key] = std::move(new_pose_log);
    }
  }

  void Log(uint64_t timestamp, std::string key, std::vector<frc::Pose3d>& value) {
    if (m_PoseArrayLogs.contains(key)) {
      m_PoseArrayLogs.at(key).Append(value);
    } else {
      auto new_pose_array_log = wpi::log::StructArrayLogEntry<frc::Pose3d>(m_Log, GetPrefixKey(key), kEntryMetadata, timestamp);

      new_pose_array_log.Append(value);
      m_PoseArrayLogs[key] = std::move(new_pose_array_log);
    }
  }

  void SetShouldUseNTTablePrefix(bool useNTTablePrefix) {
    // Use the NT/ prefix when logging to file so that when viewing .wpilog data in AdvantageScope,
    // the values can be re-used as when they are being viewed live on NetworkTables.
    if (useNTTablePrefix) {
      m_TablePrefix = "NT/";
    } else {
      m_TablePrefix = "";
    }
  }

  std::string GetPrefixKey(std::string key) {
    return m_TablePrefix + m_LogTable + "/" + key;
  }

private:
  const std::string kEntryMetadata = "{\"source\":\"BearLog\"}";

  std::string m_LogTable;
  wpi::log::DataLog& m_Log;
  std::string m_TablePrefix;

  std::unordered_map<std::string, wpi::log::BooleanLogEntry> m_BooleanLogs;
  std::unordered_map<std::string, wpi::log::DoubleArrayLogEntry> m_DoubleArrayLogs;
  std::unordered_map<std::string, wpi::log::DoubleLogEntry> m_DoubleLogs;
  std::unordered_map<std::string, wpi::log::IntegerLogEntry> m_IntegerLogs;
  std::unordered_map<std::string, wpi::log::StringArrayLogEntry> m_StringArrayLogs;
  std::unordered_map<std::string, wpi::log::StringLogEntry> m_StringLogs;
  std::unordered_map<std::string, wpi::log::StructLogEntry<frc::Pose3d>> m_PoseLogs;
  std::unordered_map<std::string, wpi::log::StructArrayLogEntry<frc::Pose3d>> m_PoseArrayLogs;
};