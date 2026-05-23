#pragma once

#include <unordered_map>

#include <frc/geometry/Pose3d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/json.h>

class NetworkTablesWriter {
public:
  const wpi::json kTopicProperties = {{"source", "\"BearLog\""}};

  NetworkTablesWriter(const std::string& logTable) {
    m_LogTable = nt::NetworkTableInstance::GetDefault().GetTable(logTable);
  }

  void Log(uint64_t timestamp, std::string key, bool value) {
    if (m_BooleanPublishers.contains(key)) {
      m_BooleanPublishers.at(key)->Set(value, timestamp);
    } else {
      nt::BooleanTopic topic = m_LogTable->GetBooleanTopic(key);
      std::unique_ptr<nt::BooleanPublisher> new_publisher = std::make_unique<nt::BooleanPublisher>(topic.Publish());
      new_publisher->Set(value, timestamp);
      topic.SetProperties(kTopicProperties);
      m_BooleanPublishers[key] = std::move(new_publisher);
    }
  }

  void Log(uint64_t timestamp, std::string key, std::span<const double> value) {
    if (m_DoubleArrayPublishers.contains(key)) {
      m_DoubleArrayPublishers.at(key)->Set(value, timestamp);
    } else {
      nt::DoubleArrayTopic topic = m_LogTable->GetDoubleArrayTopic(key);
      std::unique_ptr<nt::DoubleArrayPublisher> new_publisher = std::make_unique<nt::DoubleArrayPublisher>(topic.Publish());
      new_publisher->Set(value, timestamp);
      topic.SetProperties(kTopicProperties);
      m_DoubleArrayPublishers[key] = std::move(new_publisher);
    }
  }

  void Log(uint64_t timestamp, std::string key, double value) {
    if (m_DoublePublishers.contains(key)) {
      m_DoublePublishers.at(key)->Set(value, timestamp);
    } else {
      nt::DoubleTopic topic = m_LogTable->GetDoubleTopic(key);
      std::unique_ptr<nt::DoublePublisher> new_publisher = std::make_unique<nt::DoublePublisher>(topic.Publish());
      new_publisher->Set(value, timestamp);
      topic.SetProperties(kTopicProperties);
      m_DoublePublishers[key] = std::move(new_publisher);
    }
  }

  void Log(uint64_t timestamp, std::string key, int value) {
    if (m_IntegerPublishers.contains(key)) {
      m_IntegerPublishers.at(key)->Set(value, timestamp);
    } else {
      nt::IntegerTopic topic = m_LogTable->GetIntegerTopic(key);
      std::unique_ptr<nt::IntegerPublisher> new_publisher = std::make_unique<nt::IntegerPublisher>(topic.Publish());
      new_publisher->Set(value, timestamp);
      topic.SetProperties(kTopicProperties);
      m_IntegerPublishers[key] = std::move(new_publisher);
    }
  }

  void Log(uint64_t timestamp, std::string key, std::span<const std::string> value) {
    if (m_StringArrayPublishers.contains(key)) {
      m_StringArrayPublishers.at(key)->Set(value, timestamp);
    } else {
      nt::StringArrayTopic topic = m_LogTable->GetStringArrayTopic(key);
      std::unique_ptr<nt::StringArrayPublisher> new_publisher = std::make_unique<nt::StringArrayPublisher>(topic.Publish());
      new_publisher->Set(value, timestamp);
      topic.SetProperties(kTopicProperties);
      m_StringArrayPublishers[key] = std::move(new_publisher);
    }
  }

  void Log(uint64_t timestamp, std::string key, const std::string& value) {
    if (m_StringPublishers.contains(key)) {
      m_StringPublishers.at(key)->Set(value, timestamp);
    } else {
      nt::StringTopic topic = m_LogTable->GetStringTopic(key);
      std::unique_ptr<nt::StringPublisher> new_publisher = std::make_unique<nt::StringPublisher>(topic.Publish());
      new_publisher->Set(value, timestamp);
      topic.SetProperties(kTopicProperties);
      m_StringPublishers[key] = std::move(new_publisher);
    }
  }

  void Log(uint64_t timestamp, std::string key, const frc::Pose3d& value) {
    if (m_PosePublishers.contains(key)) {
      m_PosePublishers.at(key)->Set(value, timestamp);
    } else {
      nt::StructTopic<frc::Pose3d> topic = m_LogTable->GetStructTopic<frc::Pose3d>(key);
      std::unique_ptr<nt::StructPublisher<frc::Pose3d>> new_publisher = std::make_unique<nt::StructPublisher<frc::Pose3d>>(topic.Publish());
      new_publisher->Set(value, timestamp);
      topic.SetProperties(kTopicProperties);
      m_PosePublishers[key] = std::move(new_publisher);
    }
  }

  void Log(uint64_t timestamp, std::string key, std::vector<frc::Pose3d>& value) {
    if (m_PoseArrayPublishers.contains(key)) {
      m_PoseArrayPublishers.at(key)->Set(value, timestamp);
    } else {
      nt::StructArrayTopic<frc::Pose3d> topic = m_LogTable->GetStructArrayTopic<frc::Pose3d>(key);
      std::unique_ptr<nt::StructArrayPublisher<frc::Pose3d>> new_publisher = std::make_unique<nt::StructArrayPublisher<frc::Pose3d>>(topic.Publish());
      new_publisher->Set(value, timestamp);
      topic.SetProperties(kTopicProperties);
      m_PoseArrayPublishers[key] = std::move(new_publisher);
    }
  }

private:
  std::shared_ptr<nt::NetworkTable> m_LogTable;

  std::unordered_map<std::string, std::unique_ptr<nt::BooleanPublisher>> m_BooleanPublishers;
  std::unordered_map<std::string, std::unique_ptr<nt::DoubleArrayPublisher>> m_DoubleArrayPublishers;
  std::unordered_map<std::string, std::unique_ptr<nt::DoublePublisher>> m_DoublePublishers;
  std::unordered_map<std::string, std::unique_ptr<nt::IntegerPublisher>> m_IntegerPublishers;
  std::unordered_map<std::string, std::unique_ptr<nt::StringArrayPublisher>> m_StringArrayPublishers;
  std::unordered_map<std::string, std::unique_ptr<nt::StringPublisher>> m_StringPublishers;
  std::unordered_map<std::string, std::unique_ptr<nt::StructPublisher<frc::Pose3d>>> m_PosePublishers;
  std::unordered_map<std::string, std::unique_ptr<nt::StructArrayPublisher<frc::Pose3d>>> m_PoseArrayPublishers;
};