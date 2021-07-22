/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2019 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed under the
 * terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */

/*
 * This file has been auto-generated and should not be modified.
 */

#ifndef _KORTEX_BASE_SIMULATION_SERVICES_H_
#define _KORTEX_BASE_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/base_services_interface.h"

using namespace std;

class BaseSimulationServices : public IBaseServices
{
public:
  BaseSimulationServices(ros::NodeHandle& node_handle);

  virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request& req, kortex_driver::SetDeviceID::Response& res) override;
  virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request& req,
                             kortex_driver::SetApiOptions::Response& res) override;
  std::function<kortex_driver::CreateUserProfile::Response(const kortex_driver::CreateUserProfile::Request&)>
      CreateUserProfileHandler = nullptr;
  virtual bool CreateUserProfile(kortex_driver::CreateUserProfile::Request& req,
                                 kortex_driver::CreateUserProfile::Response& res) override;
  std::function<kortex_driver::UpdateUserProfile::Response(const kortex_driver::UpdateUserProfile::Request&)>
      UpdateUserProfileHandler = nullptr;
  virtual bool UpdateUserProfile(kortex_driver::UpdateUserProfile::Request& req,
                                 kortex_driver::UpdateUserProfile::Response& res) override;
  std::function<kortex_driver::ReadUserProfile::Response(const kortex_driver::ReadUserProfile::Request&)>
      ReadUserProfileHandler = nullptr;
  virtual bool ReadUserProfile(kortex_driver::ReadUserProfile::Request& req,
                               kortex_driver::ReadUserProfile::Response& res) override;
  std::function<kortex_driver::DeleteUserProfile::Response(const kortex_driver::DeleteUserProfile::Request&)>
      DeleteUserProfileHandler = nullptr;
  virtual bool DeleteUserProfile(kortex_driver::DeleteUserProfile::Request& req,
                                 kortex_driver::DeleteUserProfile::Response& res) override;
  std::function<kortex_driver::ReadAllUserProfiles::Response(const kortex_driver::ReadAllUserProfiles::Request&)>
      ReadAllUserProfilesHandler = nullptr;
  virtual bool ReadAllUserProfiles(kortex_driver::ReadAllUserProfiles::Request& req,
                                   kortex_driver::ReadAllUserProfiles::Response& res) override;
  std::function<kortex_driver::ReadAllUsers::Response(const kortex_driver::ReadAllUsers::Request&)> ReadAllUsersHandler =
      nullptr;
  virtual bool ReadAllUsers(kortex_driver::ReadAllUsers::Request& req,
                            kortex_driver::ReadAllUsers::Response& res) override;
  std::function<kortex_driver::ChangePassword::Response(const kortex_driver::ChangePassword::Request&)>
      ChangePasswordHandler = nullptr;
  virtual bool ChangePassword(kortex_driver::ChangePassword::Request& req,
                              kortex_driver::ChangePassword::Response& res) override;
  std::function<kortex_driver::CreateSequence::Response(const kortex_driver::CreateSequence::Request&)>
      CreateSequenceHandler = nullptr;
  virtual bool CreateSequence(kortex_driver::CreateSequence::Request& req,
                              kortex_driver::CreateSequence::Response& res) override;
  std::function<kortex_driver::UpdateSequence::Response(const kortex_driver::UpdateSequence::Request&)>
      UpdateSequenceHandler = nullptr;
  virtual bool UpdateSequence(kortex_driver::UpdateSequence::Request& req,
                              kortex_driver::UpdateSequence::Response& res) override;
  std::function<kortex_driver::ReadSequence::Response(const kortex_driver::ReadSequence::Request&)> ReadSequenceHandler =
      nullptr;
  virtual bool ReadSequence(kortex_driver::ReadSequence::Request& req,
                            kortex_driver::ReadSequence::Response& res) override;
  std::function<kortex_driver::DeleteSequence::Response(const kortex_driver::DeleteSequence::Request&)>
      DeleteSequenceHandler = nullptr;
  virtual bool DeleteSequence(kortex_driver::DeleteSequence::Request& req,
                              kortex_driver::DeleteSequence::Response& res) override;
  std::function<kortex_driver::ReadAllSequences::Response(const kortex_driver::ReadAllSequences::Request&)>
      ReadAllSequencesHandler = nullptr;
  virtual bool ReadAllSequences(kortex_driver::ReadAllSequences::Request& req,
                                kortex_driver::ReadAllSequences::Response& res) override;
  std::function<kortex_driver::PlaySequence::Response(const kortex_driver::PlaySequence::Request&)> PlaySequenceHandler =
      nullptr;
  virtual bool PlaySequence(kortex_driver::PlaySequence::Request& req,
                            kortex_driver::PlaySequence::Response& res) override;
  std::function<kortex_driver::PlayAdvancedSequence::Response(const kortex_driver::PlayAdvancedSequence::Request&)>
      PlayAdvancedSequenceHandler = nullptr;
  virtual bool PlayAdvancedSequence(kortex_driver::PlayAdvancedSequence::Request& req,
                                    kortex_driver::PlayAdvancedSequence::Response& res) override;
  std::function<kortex_driver::StopSequence::Response(const kortex_driver::StopSequence::Request&)> StopSequenceHandler =
      nullptr;
  virtual bool StopSequence(kortex_driver::StopSequence::Request& req,
                            kortex_driver::StopSequence::Response& res) override;
  std::function<kortex_driver::PauseSequence::Response(const kortex_driver::PauseSequence::Request&)>
      PauseSequenceHandler = nullptr;
  virtual bool PauseSequence(kortex_driver::PauseSequence::Request& req,
                             kortex_driver::PauseSequence::Response& res) override;
  std::function<kortex_driver::ResumeSequence::Response(const kortex_driver::ResumeSequence::Request&)>
      ResumeSequenceHandler = nullptr;
  virtual bool ResumeSequence(kortex_driver::ResumeSequence::Request& req,
                              kortex_driver::ResumeSequence::Response& res) override;
  std::function<kortex_driver::CreateProtectionZone::Response(const kortex_driver::CreateProtectionZone::Request&)>
      CreateProtectionZoneHandler = nullptr;
  virtual bool CreateProtectionZone(kortex_driver::CreateProtectionZone::Request& req,
                                    kortex_driver::CreateProtectionZone::Response& res) override;
  std::function<kortex_driver::UpdateProtectionZone::Response(const kortex_driver::UpdateProtectionZone::Request&)>
      UpdateProtectionZoneHandler = nullptr;
  virtual bool UpdateProtectionZone(kortex_driver::UpdateProtectionZone::Request& req,
                                    kortex_driver::UpdateProtectionZone::Response& res) override;
  std::function<kortex_driver::ReadProtectionZone::Response(const kortex_driver::ReadProtectionZone::Request&)>
      ReadProtectionZoneHandler = nullptr;
  virtual bool ReadProtectionZone(kortex_driver::ReadProtectionZone::Request& req,
                                  kortex_driver::ReadProtectionZone::Response& res) override;
  std::function<kortex_driver::DeleteProtectionZone::Response(const kortex_driver::DeleteProtectionZone::Request&)>
      DeleteProtectionZoneHandler = nullptr;
  virtual bool DeleteProtectionZone(kortex_driver::DeleteProtectionZone::Request& req,
                                    kortex_driver::DeleteProtectionZone::Response& res) override;
  std::function<kortex_driver::ReadAllProtectionZones::Response(const kortex_driver::ReadAllProtectionZones::Request&)>
      ReadAllProtectionZonesHandler = nullptr;
  virtual bool ReadAllProtectionZones(kortex_driver::ReadAllProtectionZones::Request& req,
                                      kortex_driver::ReadAllProtectionZones::Response& res) override;
  std::function<kortex_driver::CreateMapping::Response(const kortex_driver::CreateMapping::Request&)>
      CreateMappingHandler = nullptr;
  virtual bool CreateMapping(kortex_driver::CreateMapping::Request& req,
                             kortex_driver::CreateMapping::Response& res) override;
  std::function<kortex_driver::ReadMapping::Response(const kortex_driver::ReadMapping::Request&)> ReadMappingHandler =
      nullptr;
  virtual bool ReadMapping(kortex_driver::ReadMapping::Request& req, kortex_driver::ReadMapping::Response& res) override;
  std::function<kortex_driver::UpdateMapping::Response(const kortex_driver::UpdateMapping::Request&)>
      UpdateMappingHandler = nullptr;
  virtual bool UpdateMapping(kortex_driver::UpdateMapping::Request& req,
                             kortex_driver::UpdateMapping::Response& res) override;
  std::function<kortex_driver::DeleteMapping::Response(const kortex_driver::DeleteMapping::Request&)>
      DeleteMappingHandler = nullptr;
  virtual bool DeleteMapping(kortex_driver::DeleteMapping::Request& req,
                             kortex_driver::DeleteMapping::Response& res) override;
  std::function<kortex_driver::ReadAllMappings::Response(const kortex_driver::ReadAllMappings::Request&)>
      ReadAllMappingsHandler = nullptr;
  virtual bool ReadAllMappings(kortex_driver::ReadAllMappings::Request& req,
                               kortex_driver::ReadAllMappings::Response& res) override;
  std::function<kortex_driver::CreateMap::Response(const kortex_driver::CreateMap::Request&)> CreateMapHandler =
      nullptr;
  virtual bool CreateMap(kortex_driver::CreateMap::Request& req, kortex_driver::CreateMap::Response& res) override;
  std::function<kortex_driver::ReadMap::Response(const kortex_driver::ReadMap::Request&)> ReadMapHandler = nullptr;
  virtual bool ReadMap(kortex_driver::ReadMap::Request& req, kortex_driver::ReadMap::Response& res) override;
  std::function<kortex_driver::UpdateMap::Response(const kortex_driver::UpdateMap::Request&)> UpdateMapHandler =
      nullptr;
  virtual bool UpdateMap(kortex_driver::UpdateMap::Request& req, kortex_driver::UpdateMap::Response& res) override;
  std::function<kortex_driver::DeleteMap::Response(const kortex_driver::DeleteMap::Request&)> DeleteMapHandler =
      nullptr;
  virtual bool DeleteMap(kortex_driver::DeleteMap::Request& req, kortex_driver::DeleteMap::Response& res) override;
  std::function<kortex_driver::ReadAllMaps::Response(const kortex_driver::ReadAllMaps::Request&)> ReadAllMapsHandler =
      nullptr;
  virtual bool ReadAllMaps(kortex_driver::ReadAllMaps::Request& req, kortex_driver::ReadAllMaps::Response& res) override;
  std::function<kortex_driver::ActivateMap::Response(const kortex_driver::ActivateMap::Request&)> ActivateMapHandler =
      nullptr;
  virtual bool ActivateMap(kortex_driver::ActivateMap::Request& req, kortex_driver::ActivateMap::Response& res) override;
  std::function<kortex_driver::CreateAction::Response(const kortex_driver::CreateAction::Request&)> CreateActionHandler =
      nullptr;
  virtual bool CreateAction(kortex_driver::CreateAction::Request& req,
                            kortex_driver::CreateAction::Response& res) override;
  std::function<kortex_driver::ReadAction::Response(const kortex_driver::ReadAction::Request&)> ReadActionHandler =
      nullptr;
  virtual bool ReadAction(kortex_driver::ReadAction::Request& req, kortex_driver::ReadAction::Response& res) override;
  std::function<kortex_driver::ReadAllActions::Response(const kortex_driver::ReadAllActions::Request&)>
      ReadAllActionsHandler = nullptr;
  virtual bool ReadAllActions(kortex_driver::ReadAllActions::Request& req,
                              kortex_driver::ReadAllActions::Response& res) override;
  std::function<kortex_driver::DeleteAction::Response(const kortex_driver::DeleteAction::Request&)> DeleteActionHandler =
      nullptr;
  virtual bool DeleteAction(kortex_driver::DeleteAction::Request& req,
                            kortex_driver::DeleteAction::Response& res) override;
  std::function<kortex_driver::UpdateAction::Response(const kortex_driver::UpdateAction::Request&)> UpdateActionHandler =
      nullptr;
  virtual bool UpdateAction(kortex_driver::UpdateAction::Request& req,
                            kortex_driver::UpdateAction::Response& res) override;
  std::function<kortex_driver::ExecuteActionFromReference::Response(
      const kortex_driver::ExecuteActionFromReference::Request&)>
      ExecuteActionFromReferenceHandler = nullptr;
  virtual bool ExecuteActionFromReference(kortex_driver::ExecuteActionFromReference::Request& req,
                                          kortex_driver::ExecuteActionFromReference::Response& res) override;
  std::function<kortex_driver::ExecuteAction::Response(const kortex_driver::ExecuteAction::Request&)>
      ExecuteActionHandler = nullptr;
  virtual bool ExecuteAction(kortex_driver::ExecuteAction::Request& req,
                             kortex_driver::ExecuteAction::Response& res) override;
  std::function<kortex_driver::PauseAction::Response(const kortex_driver::PauseAction::Request&)> PauseActionHandler =
      nullptr;
  virtual bool PauseAction(kortex_driver::PauseAction::Request& req, kortex_driver::PauseAction::Response& res) override;
  std::function<kortex_driver::StopAction::Response(const kortex_driver::StopAction::Request&)> StopActionHandler =
      nullptr;
  virtual bool StopAction(kortex_driver::StopAction::Request& req, kortex_driver::StopAction::Response& res) override;
  std::function<kortex_driver::ResumeAction::Response(const kortex_driver::ResumeAction::Request&)> ResumeActionHandler =
      nullptr;
  virtual bool ResumeAction(kortex_driver::ResumeAction::Request& req,
                            kortex_driver::ResumeAction::Response& res) override;
  std::function<kortex_driver::GetIPv4Configuration::Response(const kortex_driver::GetIPv4Configuration::Request&)>
      GetIPv4ConfigurationHandler = nullptr;
  virtual bool GetIPv4Configuration(kortex_driver::GetIPv4Configuration::Request& req,
                                    kortex_driver::GetIPv4Configuration::Response& res) override;
  std::function<kortex_driver::SetIPv4Configuration::Response(const kortex_driver::SetIPv4Configuration::Request&)>
      SetIPv4ConfigurationHandler = nullptr;
  virtual bool SetIPv4Configuration(kortex_driver::SetIPv4Configuration::Request& req,
                                    kortex_driver::SetIPv4Configuration::Response& res) override;
  std::function<kortex_driver::SetCommunicationInterfaceEnable::Response(
      const kortex_driver::SetCommunicationInterfaceEnable::Request&)>
      SetCommunicationInterfaceEnableHandler = nullptr;
  virtual bool SetCommunicationInterfaceEnable(kortex_driver::SetCommunicationInterfaceEnable::Request& req,
                                               kortex_driver::SetCommunicationInterfaceEnable::Response& res) override;
  std::function<kortex_driver::IsCommunicationInterfaceEnable::Response(
      const kortex_driver::IsCommunicationInterfaceEnable::Request&)>
      IsCommunicationInterfaceEnableHandler = nullptr;
  virtual bool IsCommunicationInterfaceEnable(kortex_driver::IsCommunicationInterfaceEnable::Request& req,
                                              kortex_driver::IsCommunicationInterfaceEnable::Response& res) override;
  std::function<kortex_driver::GetAvailableWifi::Response(const kortex_driver::GetAvailableWifi::Request&)>
      GetAvailableWifiHandler = nullptr;
  virtual bool GetAvailableWifi(kortex_driver::GetAvailableWifi::Request& req,
                                kortex_driver::GetAvailableWifi::Response& res) override;
  std::function<kortex_driver::GetWifiInformation::Response(const kortex_driver::GetWifiInformation::Request&)>
      GetWifiInformationHandler = nullptr;
  virtual bool GetWifiInformation(kortex_driver::GetWifiInformation::Request& req,
                                  kortex_driver::GetWifiInformation::Response& res) override;
  std::function<kortex_driver::AddWifiConfiguration::Response(const kortex_driver::AddWifiConfiguration::Request&)>
      AddWifiConfigurationHandler = nullptr;
  virtual bool AddWifiConfiguration(kortex_driver::AddWifiConfiguration::Request& req,
                                    kortex_driver::AddWifiConfiguration::Response& res) override;
  std::function<kortex_driver::DeleteWifiConfiguration::Response(const kortex_driver::DeleteWifiConfiguration::Request&)>
      DeleteWifiConfigurationHandler = nullptr;
  virtual bool DeleteWifiConfiguration(kortex_driver::DeleteWifiConfiguration::Request& req,
                                       kortex_driver::DeleteWifiConfiguration::Response& res) override;
  std::function<kortex_driver::GetAllConfiguredWifis::Response(const kortex_driver::GetAllConfiguredWifis::Request&)>
      GetAllConfiguredWifisHandler = nullptr;
  virtual bool GetAllConfiguredWifis(kortex_driver::GetAllConfiguredWifis::Request& req,
                                     kortex_driver::GetAllConfiguredWifis::Response& res) override;
  std::function<kortex_driver::ConnectWifi::Response(const kortex_driver::ConnectWifi::Request&)> ConnectWifiHandler =
      nullptr;
  virtual bool ConnectWifi(kortex_driver::ConnectWifi::Request& req, kortex_driver::ConnectWifi::Response& res) override;
  std::function<kortex_driver::DisconnectWifi::Response(const kortex_driver::DisconnectWifi::Request&)>
      DisconnectWifiHandler = nullptr;
  virtual bool DisconnectWifi(kortex_driver::DisconnectWifi::Request& req,
                              kortex_driver::DisconnectWifi::Response& res) override;
  std::function<kortex_driver::GetConnectedWifiInformation::Response(
      const kortex_driver::GetConnectedWifiInformation::Request&)>
      GetConnectedWifiInformationHandler = nullptr;
  virtual bool GetConnectedWifiInformation(kortex_driver::GetConnectedWifiInformation::Request& req,
                                           kortex_driver::GetConnectedWifiInformation::Response& res) override;
  std::function<kortex_driver::Base_Unsubscribe::Response(const kortex_driver::Base_Unsubscribe::Request&)>
      Base_UnsubscribeHandler = nullptr;
  virtual bool Base_Unsubscribe(kortex_driver::Base_Unsubscribe::Request& req,
                                kortex_driver::Base_Unsubscribe::Response& res) override;
  std::function<kortex_driver::OnNotificationConfigurationChangeTopic::Response(
      const kortex_driver::OnNotificationConfigurationChangeTopic::Request&)>
      OnNotificationConfigurationChangeTopicHandler = nullptr;
  virtual bool
  OnNotificationConfigurationChangeTopic(kortex_driver::OnNotificationConfigurationChangeTopic::Request& req,
                                         kortex_driver::OnNotificationConfigurationChangeTopic::Response& res) override;
  virtual void cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif) override;
  std::function<kortex_driver::OnNotificationMappingInfoTopic::Response(
      const kortex_driver::OnNotificationMappingInfoTopic::Request&)>
      OnNotificationMappingInfoTopicHandler = nullptr;
  virtual bool OnNotificationMappingInfoTopic(kortex_driver::OnNotificationMappingInfoTopic::Request& req,
                                              kortex_driver::OnNotificationMappingInfoTopic::Response& res) override;
  virtual void cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif) override;
  std::function<kortex_driver::Base_OnNotificationControlModeTopic::Response(
      const kortex_driver::Base_OnNotificationControlModeTopic::Request&)>
      Base_OnNotificationControlModeTopicHandler = nullptr;
  virtual bool
  Base_OnNotificationControlModeTopic(kortex_driver::Base_OnNotificationControlModeTopic::Request& req,
                                      kortex_driver::Base_OnNotificationControlModeTopic::Response& res) override;
  virtual void cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif) override;
  std::function<kortex_driver::OnNotificationOperatingModeTopic::Response(
      const kortex_driver::OnNotificationOperatingModeTopic::Request&)>
      OnNotificationOperatingModeTopicHandler = nullptr;
  virtual bool OnNotificationOperatingModeTopic(kortex_driver::OnNotificationOperatingModeTopic::Request& req,
                                                kortex_driver::OnNotificationOperatingModeTopic::Response& res) override;
  virtual void cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif) override;
  std::function<kortex_driver::OnNotificationSequenceInfoTopic::Response(
      const kortex_driver::OnNotificationSequenceInfoTopic::Request&)>
      OnNotificationSequenceInfoTopicHandler = nullptr;
  virtual bool OnNotificationSequenceInfoTopic(kortex_driver::OnNotificationSequenceInfoTopic::Request& req,
                                               kortex_driver::OnNotificationSequenceInfoTopic::Response& res) override;
  virtual void cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif) override;
  std::function<kortex_driver::OnNotificationProtectionZoneTopic::Response(
      const kortex_driver::OnNotificationProtectionZoneTopic::Request&)>
      OnNotificationProtectionZoneTopicHandler = nullptr;
  virtual bool
  OnNotificationProtectionZoneTopic(kortex_driver::OnNotificationProtectionZoneTopic::Request& req,
                                    kortex_driver::OnNotificationProtectionZoneTopic::Response& res) override;
  virtual void cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif) override;
  std::function<kortex_driver::OnNotificationUserTopic::Response(const kortex_driver::OnNotificationUserTopic::Request&)>
      OnNotificationUserTopicHandler = nullptr;
  virtual bool OnNotificationUserTopic(kortex_driver::OnNotificationUserTopic::Request& req,
                                       kortex_driver::OnNotificationUserTopic::Response& res) override;
  virtual void cb_UserTopic(Kinova::Api::Base::UserNotification notif) override;
  std::function<kortex_driver::OnNotificationControllerTopic::Response(
      const kortex_driver::OnNotificationControllerTopic::Request&)>
      OnNotificationControllerTopicHandler = nullptr;
  virtual bool OnNotificationControllerTopic(kortex_driver::OnNotificationControllerTopic::Request& req,
                                             kortex_driver::OnNotificationControllerTopic::Response& res) override;
  virtual void cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif) override;
  std::function<kortex_driver::OnNotificationActionTopic::Response(
      const kortex_driver::OnNotificationActionTopic::Request&)>
      OnNotificationActionTopicHandler = nullptr;
  virtual bool OnNotificationActionTopic(kortex_driver::OnNotificationActionTopic::Request& req,
                                         kortex_driver::OnNotificationActionTopic::Response& res) override;
  virtual void cb_ActionTopic(Kinova::Api::Base::ActionNotification notif) override;
  std::function<kortex_driver::OnNotificationRobotEventTopic::Response(
      const kortex_driver::OnNotificationRobotEventTopic::Request&)>
      OnNotificationRobotEventTopicHandler = nullptr;
  virtual bool OnNotificationRobotEventTopic(kortex_driver::OnNotificationRobotEventTopic::Request& req,
                                             kortex_driver::OnNotificationRobotEventTopic::Response& res) override;
  virtual void cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif) override;
  std::function<kortex_driver::PlayCartesianTrajectory::Response(const kortex_driver::PlayCartesianTrajectory::Request&)>
      PlayCartesianTrajectoryHandler = nullptr;
  virtual bool PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request& req,
                                       kortex_driver::PlayCartesianTrajectory::Response& res) override;
  std::function<kortex_driver::PlayCartesianTrajectoryPosition::Response(
      const kortex_driver::PlayCartesianTrajectoryPosition::Request&)>
      PlayCartesianTrajectoryPositionHandler = nullptr;
  virtual bool PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request& req,
                                               kortex_driver::PlayCartesianTrajectoryPosition::Response& res) override;
  std::function<kortex_driver::PlayCartesianTrajectoryOrientation::Response(
      const kortex_driver::PlayCartesianTrajectoryOrientation::Request&)>
      PlayCartesianTrajectoryOrientationHandler = nullptr;
  virtual bool
  PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request& req,
                                     kortex_driver::PlayCartesianTrajectoryOrientation::Response& res) override;
  std::function<kortex_driver::Stop::Response(const kortex_driver::Stop::Request&)> StopHandler = nullptr;
  virtual bool Stop(kortex_driver::Stop::Request& req, kortex_driver::Stop::Response& res) override;
  std::function<kortex_driver::GetMeasuredCartesianPose::Response(const kortex_driver::GetMeasuredCartesianPose::Request&)>
      GetMeasuredCartesianPoseHandler = nullptr;
  virtual bool GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request& req,
                                        kortex_driver::GetMeasuredCartesianPose::Response& res) override;
  std::function<kortex_driver::SendWrenchCommand::Response(const kortex_driver::SendWrenchCommand::Request&)>
      SendWrenchCommandHandler = nullptr;
  virtual bool SendWrenchCommand(kortex_driver::SendWrenchCommand::Request& req,
                                 kortex_driver::SendWrenchCommand::Response& res) override;
  std::function<kortex_driver::SendWrenchJoystickCommand::Response(
      const kortex_driver::SendWrenchJoystickCommand::Request&)>
      SendWrenchJoystickCommandHandler = nullptr;
  virtual bool SendWrenchJoystickCommand(kortex_driver::SendWrenchJoystickCommand::Request& req,
                                         kortex_driver::SendWrenchJoystickCommand::Response& res) override;
  std::function<kortex_driver::SendTwistJoystickCommand::Response(const kortex_driver::SendTwistJoystickCommand::Request&)>
      SendTwistJoystickCommandHandler = nullptr;
  virtual bool SendTwistJoystickCommand(kortex_driver::SendTwistJoystickCommand::Request& req,
                                        kortex_driver::SendTwistJoystickCommand::Response& res) override;
  std::function<kortex_driver::SendTwistCommand::Response(const kortex_driver::SendTwistCommand::Request&)>
      SendTwistCommandHandler = nullptr;
  virtual bool SendTwistCommand(kortex_driver::SendTwistCommand::Request& req,
                                kortex_driver::SendTwistCommand::Response& res) override;
  std::function<kortex_driver::PlayJointTrajectory::Response(const kortex_driver::PlayJointTrajectory::Request&)>
      PlayJointTrajectoryHandler = nullptr;
  virtual bool PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request& req,
                                   kortex_driver::PlayJointTrajectory::Response& res) override;
  std::function<kortex_driver::PlaySelectedJointTrajectory::Response(
      const kortex_driver::PlaySelectedJointTrajectory::Request&)>
      PlaySelectedJointTrajectoryHandler = nullptr;
  virtual bool PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request& req,
                                           kortex_driver::PlaySelectedJointTrajectory::Response& res) override;
  std::function<kortex_driver::GetMeasuredJointAngles::Response(const kortex_driver::GetMeasuredJointAngles::Request&)>
      GetMeasuredJointAnglesHandler = nullptr;
  virtual bool GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request& req,
                                      kortex_driver::GetMeasuredJointAngles::Response& res) override;
  std::function<kortex_driver::SendJointSpeedsCommand::Response(const kortex_driver::SendJointSpeedsCommand::Request&)>
      SendJointSpeedsCommandHandler = nullptr;
  virtual bool SendJointSpeedsCommand(kortex_driver::SendJointSpeedsCommand::Request& req,
                                      kortex_driver::SendJointSpeedsCommand::Response& res) override;
  std::function<kortex_driver::SendSelectedJointSpeedCommand::Response(
      const kortex_driver::SendSelectedJointSpeedCommand::Request&)>
      SendSelectedJointSpeedCommandHandler = nullptr;
  virtual bool SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request& req,
                                             kortex_driver::SendSelectedJointSpeedCommand::Response& res) override;
  std::function<kortex_driver::SendGripperCommand::Response(const kortex_driver::SendGripperCommand::Request&)>
      SendGripperCommandHandler = nullptr;
  virtual bool SendGripperCommand(kortex_driver::SendGripperCommand::Request& req,
                                  kortex_driver::SendGripperCommand::Response& res) override;
  std::function<kortex_driver::GetMeasuredGripperMovement::Response(
      const kortex_driver::GetMeasuredGripperMovement::Request&)>
      GetMeasuredGripperMovementHandler = nullptr;
  virtual bool GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request& req,
                                          kortex_driver::GetMeasuredGripperMovement::Response& res) override;
  std::function<kortex_driver::SetAdmittance::Response(const kortex_driver::SetAdmittance::Request&)>
      SetAdmittanceHandler = nullptr;
  virtual bool SetAdmittance(kortex_driver::SetAdmittance::Request& req,
                             kortex_driver::SetAdmittance::Response& res) override;
  std::function<kortex_driver::SetOperatingMode::Response(const kortex_driver::SetOperatingMode::Request&)>
      SetOperatingModeHandler = nullptr;
  virtual bool SetOperatingMode(kortex_driver::SetOperatingMode::Request& req,
                                kortex_driver::SetOperatingMode::Response& res) override;
  std::function<kortex_driver::ApplyEmergencyStop::Response(const kortex_driver::ApplyEmergencyStop::Request&)>
      ApplyEmergencyStopHandler = nullptr;
  virtual bool ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request& req,
                                  kortex_driver::ApplyEmergencyStop::Response& res) override;
  std::function<kortex_driver::Base_ClearFaults::Response(const kortex_driver::Base_ClearFaults::Request&)>
      Base_ClearFaultsHandler = nullptr;
  virtual bool Base_ClearFaults(kortex_driver::Base_ClearFaults::Request& req,
                                kortex_driver::Base_ClearFaults::Response& res) override;
  std::function<kortex_driver::Base_GetControlMode::Response(const kortex_driver::Base_GetControlMode::Request&)>
      Base_GetControlModeHandler = nullptr;
  virtual bool Base_GetControlMode(kortex_driver::Base_GetControlMode::Request& req,
                                   kortex_driver::Base_GetControlMode::Response& res) override;
  std::function<kortex_driver::GetOperatingMode::Response(const kortex_driver::GetOperatingMode::Request&)>
      GetOperatingModeHandler = nullptr;
  virtual bool GetOperatingMode(kortex_driver::GetOperatingMode::Request& req,
                                kortex_driver::GetOperatingMode::Response& res) override;
  std::function<kortex_driver::SetServoingMode::Response(const kortex_driver::SetServoingMode::Request&)>
      SetServoingModeHandler = nullptr;
  virtual bool SetServoingMode(kortex_driver::SetServoingMode::Request& req,
                               kortex_driver::SetServoingMode::Response& res) override;
  std::function<kortex_driver::GetServoingMode::Response(const kortex_driver::GetServoingMode::Request&)>
      GetServoingModeHandler = nullptr;
  virtual bool GetServoingMode(kortex_driver::GetServoingMode::Request& req,
                               kortex_driver::GetServoingMode::Response& res) override;
  std::function<kortex_driver::OnNotificationServoingModeTopic::Response(
      const kortex_driver::OnNotificationServoingModeTopic::Request&)>
      OnNotificationServoingModeTopicHandler = nullptr;
  virtual bool OnNotificationServoingModeTopic(kortex_driver::OnNotificationServoingModeTopic::Request& req,
                                               kortex_driver::OnNotificationServoingModeTopic::Response& res) override;
  virtual void cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif) override;
  std::function<kortex_driver::RestoreFactorySettings::Response(const kortex_driver::RestoreFactorySettings::Request&)>
      RestoreFactorySettingsHandler = nullptr;
  virtual bool RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request& req,
                                      kortex_driver::RestoreFactorySettings::Response& res) override;
  std::function<kortex_driver::OnNotificationFactoryTopic::Response(
      const kortex_driver::OnNotificationFactoryTopic::Request&)>
      OnNotificationFactoryTopicHandler = nullptr;
  virtual bool OnNotificationFactoryTopic(kortex_driver::OnNotificationFactoryTopic::Request& req,
                                          kortex_driver::OnNotificationFactoryTopic::Response& res) override;
  virtual void cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif) override;
  std::function<kortex_driver::GetAllConnectedControllers::Response(
      const kortex_driver::GetAllConnectedControllers::Request&)>
      GetAllConnectedControllersHandler = nullptr;
  virtual bool GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request& req,
                                          kortex_driver::GetAllConnectedControllers::Response& res) override;
  std::function<kortex_driver::GetControllerState::Response(const kortex_driver::GetControllerState::Request&)>
      GetControllerStateHandler = nullptr;
  virtual bool GetControllerState(kortex_driver::GetControllerState::Request& req,
                                  kortex_driver::GetControllerState::Response& res) override;
  std::function<kortex_driver::GetActuatorCount::Response(const kortex_driver::GetActuatorCount::Request&)>
      GetActuatorCountHandler = nullptr;
  virtual bool GetActuatorCount(kortex_driver::GetActuatorCount::Request& req,
                                kortex_driver::GetActuatorCount::Response& res) override;
  std::function<kortex_driver::StartWifiScan::Response(const kortex_driver::StartWifiScan::Request&)>
      StartWifiScanHandler = nullptr;
  virtual bool StartWifiScan(kortex_driver::StartWifiScan::Request& req,
                             kortex_driver::StartWifiScan::Response& res) override;
  std::function<kortex_driver::GetConfiguredWifi::Response(const kortex_driver::GetConfiguredWifi::Request&)>
      GetConfiguredWifiHandler = nullptr;
  virtual bool GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request& req,
                                 kortex_driver::GetConfiguredWifi::Response& res) override;
  std::function<kortex_driver::OnNotificationNetworkTopic::Response(
      const kortex_driver::OnNotificationNetworkTopic::Request&)>
      OnNotificationNetworkTopicHandler = nullptr;
  virtual bool OnNotificationNetworkTopic(kortex_driver::OnNotificationNetworkTopic::Request& req,
                                          kortex_driver::OnNotificationNetworkTopic::Response& res) override;
  virtual void cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif) override;
  std::function<kortex_driver::GetArmState::Response(const kortex_driver::GetArmState::Request&)> GetArmStateHandler =
      nullptr;
  virtual bool GetArmState(kortex_driver::GetArmState::Request& req, kortex_driver::GetArmState::Response& res) override;
  std::function<kortex_driver::OnNotificationArmStateTopic::Response(
      const kortex_driver::OnNotificationArmStateTopic::Request&)>
      OnNotificationArmStateTopicHandler = nullptr;
  virtual bool OnNotificationArmStateTopic(kortex_driver::OnNotificationArmStateTopic::Request& req,
                                           kortex_driver::OnNotificationArmStateTopic::Response& res) override;
  virtual void cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif) override;
  std::function<kortex_driver::GetIPv4Information::Response(const kortex_driver::GetIPv4Information::Request&)>
      GetIPv4InformationHandler = nullptr;
  virtual bool GetIPv4Information(kortex_driver::GetIPv4Information::Request& req,
                                  kortex_driver::GetIPv4Information::Response& res) override;
  std::function<kortex_driver::SetWifiCountryCode::Response(const kortex_driver::SetWifiCountryCode::Request&)>
      SetWifiCountryCodeHandler = nullptr;
  virtual bool SetWifiCountryCode(kortex_driver::SetWifiCountryCode::Request& req,
                                  kortex_driver::SetWifiCountryCode::Response& res) override;
  std::function<kortex_driver::GetWifiCountryCode::Response(const kortex_driver::GetWifiCountryCode::Request&)>
      GetWifiCountryCodeHandler = nullptr;
  virtual bool GetWifiCountryCode(kortex_driver::GetWifiCountryCode::Request& req,
                                  kortex_driver::GetWifiCountryCode::Response& res) override;
  std::function<kortex_driver::Base_SetCapSenseConfig::Response(const kortex_driver::Base_SetCapSenseConfig::Request&)>
      Base_SetCapSenseConfigHandler = nullptr;
  virtual bool Base_SetCapSenseConfig(kortex_driver::Base_SetCapSenseConfig::Request& req,
                                      kortex_driver::Base_SetCapSenseConfig::Response& res) override;
  std::function<kortex_driver::Base_GetCapSenseConfig::Response(const kortex_driver::Base_GetCapSenseConfig::Request&)>
      Base_GetCapSenseConfigHandler = nullptr;
  virtual bool Base_GetCapSenseConfig(kortex_driver::Base_GetCapSenseConfig::Request& req,
                                      kortex_driver::Base_GetCapSenseConfig::Response& res) override;
  std::function<kortex_driver::GetAllJointsSpeedHardLimitation::Response(
      const kortex_driver::GetAllJointsSpeedHardLimitation::Request&)>
      GetAllJointsSpeedHardLimitationHandler = nullptr;
  virtual bool GetAllJointsSpeedHardLimitation(kortex_driver::GetAllJointsSpeedHardLimitation::Request& req,
                                               kortex_driver::GetAllJointsSpeedHardLimitation::Response& res) override;
  std::function<kortex_driver::GetAllJointsTorqueHardLimitation::Response(
      const kortex_driver::GetAllJointsTorqueHardLimitation::Request&)>
      GetAllJointsTorqueHardLimitationHandler = nullptr;
  virtual bool GetAllJointsTorqueHardLimitation(kortex_driver::GetAllJointsTorqueHardLimitation::Request& req,
                                                kortex_driver::GetAllJointsTorqueHardLimitation::Response& res) override;
  std::function<kortex_driver::GetTwistHardLimitation::Response(const kortex_driver::GetTwistHardLimitation::Request&)>
      GetTwistHardLimitationHandler = nullptr;
  virtual bool GetTwistHardLimitation(kortex_driver::GetTwistHardLimitation::Request& req,
                                      kortex_driver::GetTwistHardLimitation::Response& res) override;
  std::function<kortex_driver::GetWrenchHardLimitation::Response(const kortex_driver::GetWrenchHardLimitation::Request&)>
      GetWrenchHardLimitationHandler = nullptr;
  virtual bool GetWrenchHardLimitation(kortex_driver::GetWrenchHardLimitation::Request& req,
                                       kortex_driver::GetWrenchHardLimitation::Response& res) override;
  std::function<kortex_driver::SendJointSpeedsJoystickCommand::Response(
      const kortex_driver::SendJointSpeedsJoystickCommand::Request&)>
      SendJointSpeedsJoystickCommandHandler = nullptr;
  virtual bool SendJointSpeedsJoystickCommand(kortex_driver::SendJointSpeedsJoystickCommand::Request& req,
                                              kortex_driver::SendJointSpeedsJoystickCommand::Response& res) override;
  std::function<kortex_driver::SendSelectedJointSpeedJoystickCommand::Response(
      const kortex_driver::SendSelectedJointSpeedJoystickCommand::Request&)>
      SendSelectedJointSpeedJoystickCommandHandler = nullptr;
  virtual bool
  SendSelectedJointSpeedJoystickCommand(kortex_driver::SendSelectedJointSpeedJoystickCommand::Request& req,
                                        kortex_driver::SendSelectedJointSpeedJoystickCommand::Response& res) override;
  std::function<kortex_driver::EnableBridge::Response(const kortex_driver::EnableBridge::Request&)> EnableBridgeHandler =
      nullptr;
  virtual bool EnableBridge(kortex_driver::EnableBridge::Request& req,
                            kortex_driver::EnableBridge::Response& res) override;
  std::function<kortex_driver::DisableBridge::Response(const kortex_driver::DisableBridge::Request&)>
      DisableBridgeHandler = nullptr;
  virtual bool DisableBridge(kortex_driver::DisableBridge::Request& req,
                             kortex_driver::DisableBridge::Response& res) override;
  std::function<kortex_driver::GetBridgeList::Response(const kortex_driver::GetBridgeList::Request&)>
      GetBridgeListHandler = nullptr;
  virtual bool GetBridgeList(kortex_driver::GetBridgeList::Request& req,
                             kortex_driver::GetBridgeList::Response& res) override;
  std::function<kortex_driver::GetBridgeConfig::Response(const kortex_driver::GetBridgeConfig::Request&)>
      GetBridgeConfigHandler = nullptr;
  virtual bool GetBridgeConfig(kortex_driver::GetBridgeConfig::Request& req,
                               kortex_driver::GetBridgeConfig::Response& res) override;
  std::function<kortex_driver::PlayPreComputedJointTrajectory::Response(
      const kortex_driver::PlayPreComputedJointTrajectory::Request&)>
      PlayPreComputedJointTrajectoryHandler = nullptr;
  virtual bool PlayPreComputedJointTrajectory(kortex_driver::PlayPreComputedJointTrajectory::Request& req,
                                              kortex_driver::PlayPreComputedJointTrajectory::Response& res) override;
  std::function<kortex_driver::GetProductConfiguration::Response(const kortex_driver::GetProductConfiguration::Request&)>
      GetProductConfigurationHandler = nullptr;
  virtual bool GetProductConfiguration(kortex_driver::GetProductConfiguration::Request& req,
                                       kortex_driver::GetProductConfiguration::Response& res) override;
  std::function<kortex_driver::UpdateEndEffectorTypeConfiguration::Response(
      const kortex_driver::UpdateEndEffectorTypeConfiguration::Request&)>
      UpdateEndEffectorTypeConfigurationHandler = nullptr;
  virtual bool
  UpdateEndEffectorTypeConfiguration(kortex_driver::UpdateEndEffectorTypeConfiguration::Request& req,
                                     kortex_driver::UpdateEndEffectorTypeConfiguration::Response& res) override;
  std::function<kortex_driver::RestoreFactoryProductConfiguration::Response(
      const kortex_driver::RestoreFactoryProductConfiguration::Request&)>
      RestoreFactoryProductConfigurationHandler = nullptr;
  virtual bool
  RestoreFactoryProductConfiguration(kortex_driver::RestoreFactoryProductConfiguration::Request& req,
                                     kortex_driver::RestoreFactoryProductConfiguration::Response& res) override;
  std::function<kortex_driver::GetTrajectoryErrorReport::Response(const kortex_driver::GetTrajectoryErrorReport::Request&)>
      GetTrajectoryErrorReportHandler = nullptr;
  virtual bool GetTrajectoryErrorReport(kortex_driver::GetTrajectoryErrorReport::Request& req,
                                        kortex_driver::GetTrajectoryErrorReport::Response& res) override;
  std::function<kortex_driver::GetAllJointsSpeedSoftLimitation::Response(
      const kortex_driver::GetAllJointsSpeedSoftLimitation::Request&)>
      GetAllJointsSpeedSoftLimitationHandler = nullptr;
  virtual bool GetAllJointsSpeedSoftLimitation(kortex_driver::GetAllJointsSpeedSoftLimitation::Request& req,
                                               kortex_driver::GetAllJointsSpeedSoftLimitation::Response& res) override;
  std::function<kortex_driver::GetAllJointsTorqueSoftLimitation::Response(
      const kortex_driver::GetAllJointsTorqueSoftLimitation::Request&)>
      GetAllJointsTorqueSoftLimitationHandler = nullptr;
  virtual bool GetAllJointsTorqueSoftLimitation(kortex_driver::GetAllJointsTorqueSoftLimitation::Request& req,
                                                kortex_driver::GetAllJointsTorqueSoftLimitation::Response& res) override;
  std::function<kortex_driver::GetTwistSoftLimitation::Response(const kortex_driver::GetTwistSoftLimitation::Request&)>
      GetTwistSoftLimitationHandler = nullptr;
  virtual bool GetTwistSoftLimitation(kortex_driver::GetTwistSoftLimitation::Request& req,
                                      kortex_driver::GetTwistSoftLimitation::Response& res) override;
  std::function<kortex_driver::GetWrenchSoftLimitation::Response(const kortex_driver::GetWrenchSoftLimitation::Request&)>
      GetWrenchSoftLimitationHandler = nullptr;
  virtual bool GetWrenchSoftLimitation(kortex_driver::GetWrenchSoftLimitation::Request& req,
                                       kortex_driver::GetWrenchSoftLimitation::Response& res) override;
  std::function<kortex_driver::SetControllerConfigurationMode::Response(
      const kortex_driver::SetControllerConfigurationMode::Request&)>
      SetControllerConfigurationModeHandler = nullptr;
  virtual bool SetControllerConfigurationMode(kortex_driver::SetControllerConfigurationMode::Request& req,
                                              kortex_driver::SetControllerConfigurationMode::Response& res) override;
  std::function<kortex_driver::GetControllerConfigurationMode::Response(
      const kortex_driver::GetControllerConfigurationMode::Request&)>
      GetControllerConfigurationModeHandler = nullptr;
  virtual bool GetControllerConfigurationMode(kortex_driver::GetControllerConfigurationMode::Request& req,
                                              kortex_driver::GetControllerConfigurationMode::Response& res) override;
  std::function<kortex_driver::StartTeaching::Response(const kortex_driver::StartTeaching::Request&)>
      StartTeachingHandler = nullptr;
  virtual bool StartTeaching(kortex_driver::StartTeaching::Request& req,
                             kortex_driver::StartTeaching::Response& res) override;
  std::function<kortex_driver::StopTeaching::Response(const kortex_driver::StopTeaching::Request&)> StopTeachingHandler =
      nullptr;
  virtual bool StopTeaching(kortex_driver::StopTeaching::Request& req,
                            kortex_driver::StopTeaching::Response& res) override;
  std::function<kortex_driver::AddSequenceTasks::Response(const kortex_driver::AddSequenceTasks::Request&)>
      AddSequenceTasksHandler = nullptr;
  virtual bool AddSequenceTasks(kortex_driver::AddSequenceTasks::Request& req,
                                kortex_driver::AddSequenceTasks::Response& res) override;
  std::function<kortex_driver::UpdateSequenceTask::Response(const kortex_driver::UpdateSequenceTask::Request&)>
      UpdateSequenceTaskHandler = nullptr;
  virtual bool UpdateSequenceTask(kortex_driver::UpdateSequenceTask::Request& req,
                                  kortex_driver::UpdateSequenceTask::Response& res) override;
  std::function<kortex_driver::SwapSequenceTasks::Response(const kortex_driver::SwapSequenceTasks::Request&)>
      SwapSequenceTasksHandler = nullptr;
  virtual bool SwapSequenceTasks(kortex_driver::SwapSequenceTasks::Request& req,
                                 kortex_driver::SwapSequenceTasks::Response& res) override;
  std::function<kortex_driver::ReadSequenceTask::Response(const kortex_driver::ReadSequenceTask::Request&)>
      ReadSequenceTaskHandler = nullptr;
  virtual bool ReadSequenceTask(kortex_driver::ReadSequenceTask::Request& req,
                                kortex_driver::ReadSequenceTask::Response& res) override;
  std::function<kortex_driver::ReadAllSequenceTasks::Response(const kortex_driver::ReadAllSequenceTasks::Request&)>
      ReadAllSequenceTasksHandler = nullptr;
  virtual bool ReadAllSequenceTasks(kortex_driver::ReadAllSequenceTasks::Request& req,
                                    kortex_driver::ReadAllSequenceTasks::Response& res) override;
  std::function<kortex_driver::DeleteSequenceTask::Response(const kortex_driver::DeleteSequenceTask::Request&)>
      DeleteSequenceTaskHandler = nullptr;
  virtual bool DeleteSequenceTask(kortex_driver::DeleteSequenceTask::Request& req,
                                  kortex_driver::DeleteSequenceTask::Response& res) override;
  std::function<kortex_driver::DeleteAllSequenceTasks::Response(const kortex_driver::DeleteAllSequenceTasks::Request&)>
      DeleteAllSequenceTasksHandler = nullptr;
  virtual bool DeleteAllSequenceTasks(kortex_driver::DeleteAllSequenceTasks::Request& req,
                                      kortex_driver::DeleteAllSequenceTasks::Response& res) override;
  std::function<kortex_driver::TakeSnapshot::Response(const kortex_driver::TakeSnapshot::Request&)> TakeSnapshotHandler =
      nullptr;
  virtual bool TakeSnapshot(kortex_driver::TakeSnapshot::Request& req,
                            kortex_driver::TakeSnapshot::Response& res) override;
  std::function<kortex_driver::GetFirmwareBundleVersions::Response(
      const kortex_driver::GetFirmwareBundleVersions::Request&)>
      GetFirmwareBundleVersionsHandler = nullptr;
  virtual bool GetFirmwareBundleVersions(kortex_driver::GetFirmwareBundleVersions::Request& req,
                                         kortex_driver::GetFirmwareBundleVersions::Response& res) override;
  std::function<kortex_driver::ExecuteWaypointTrajectory::Response(
      const kortex_driver::ExecuteWaypointTrajectory::Request&)>
      ExecuteWaypointTrajectoryHandler = nullptr;
  virtual bool ExecuteWaypointTrajectory(kortex_driver::ExecuteWaypointTrajectory::Request& req,
                                         kortex_driver::ExecuteWaypointTrajectory::Response& res) override;
  std::function<kortex_driver::MoveSequenceTask::Response(const kortex_driver::MoveSequenceTask::Request&)>
      MoveSequenceTaskHandler = nullptr;
  virtual bool MoveSequenceTask(kortex_driver::MoveSequenceTask::Request& req,
                                kortex_driver::MoveSequenceTask::Response& res) override;
  std::function<kortex_driver::DuplicateMapping::Response(const kortex_driver::DuplicateMapping::Request&)>
      DuplicateMappingHandler = nullptr;
  virtual bool DuplicateMapping(kortex_driver::DuplicateMapping::Request& req,
                                kortex_driver::DuplicateMapping::Response& res) override;
  std::function<kortex_driver::DuplicateMap::Response(const kortex_driver::DuplicateMap::Request&)> DuplicateMapHandler =
      nullptr;
  virtual bool DuplicateMap(kortex_driver::DuplicateMap::Request& req,
                            kortex_driver::DuplicateMap::Response& res) override;
  std::function<kortex_driver::SetControllerConfiguration::Response(
      const kortex_driver::SetControllerConfiguration::Request&)>
      SetControllerConfigurationHandler = nullptr;
  virtual bool SetControllerConfiguration(kortex_driver::SetControllerConfiguration::Request& req,
                                          kortex_driver::SetControllerConfiguration::Response& res) override;
  std::function<kortex_driver::GetControllerConfiguration::Response(
      const kortex_driver::GetControllerConfiguration::Request&)>
      GetControllerConfigurationHandler = nullptr;
  virtual bool GetControllerConfiguration(kortex_driver::GetControllerConfiguration::Request& req,
                                          kortex_driver::GetControllerConfiguration::Response& res) override;
  std::function<kortex_driver::GetAllControllerConfigurations::Response(
      const kortex_driver::GetAllControllerConfigurations::Request&)>
      GetAllControllerConfigurationsHandler = nullptr;
  virtual bool GetAllControllerConfigurations(kortex_driver::GetAllControllerConfigurations::Request& req,
                                              kortex_driver::GetAllControllerConfigurations::Response& res) override;
  std::function<kortex_driver::ComputeForwardKinematics::Response(const kortex_driver::ComputeForwardKinematics::Request&)>
      ComputeForwardKinematicsHandler = nullptr;
  virtual bool ComputeForwardKinematics(kortex_driver::ComputeForwardKinematics::Request& req,
                                        kortex_driver::ComputeForwardKinematics::Response& res) override;
  std::function<kortex_driver::ComputeInverseKinematics::Response(const kortex_driver::ComputeInverseKinematics::Request&)>
      ComputeInverseKinematicsHandler = nullptr;
  virtual bool ComputeInverseKinematics(kortex_driver::ComputeInverseKinematics::Request& req,
                                        kortex_driver::ComputeInverseKinematics::Response& res) override;
  std::function<kortex_driver::ValidateWaypointList::Response(const kortex_driver::ValidateWaypointList::Request&)>
      ValidateWaypointListHandler = nullptr;
  virtual bool ValidateWaypointList(kortex_driver::ValidateWaypointList::Request& req,
                                    kortex_driver::ValidateWaypointList::Response& res) override;
};
#endif
