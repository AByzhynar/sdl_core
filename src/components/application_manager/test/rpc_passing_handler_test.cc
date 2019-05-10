/*
 * Copyright (c) 2019, Ford Motor Company
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of the Ford Motor Company nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gmock/gmock.h>
#include "gtest/gtest.h"

#include "application_manager/rpc_passing_handler.h"

#include "application_manager/mock_app_service_manager.h"
#include "application_manager/mock_application.h"
#include "application_manager/mock_application_manager.h"
#include "application_manager/mock_application_manager_settings.h"
#include "application_manager/mock_message_helper.h"
#include "application_manager/mock_rpc_handler.h"
#include "application_manager/mock_rpc_service.h"
#include "application_manager/policies/mock_policy_handler_interface.h"

#include <vector>
#include "application_manager/smart_object_keys.h"
#include "resumption/mock_last_state.h"
#include "smart_objects/smart_object.h"
#include "utils/semantic_version.h"

namespace test {
namespace components {
namespace application_manager_test {

namespace am = application_manager;
using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Mock;
using ::testing::NiceMock;
using ::testing::Pointee;
using ::testing::Return;
using ::testing::ReturnRef;

class RPCPassingHandlerTest : public ::testing::Test {
 public:
  RPCPassingHandlerTest()
      : mock_app_service_manager_(mock_app_manager_, mock_last_state_)
      , mock_app_ptr_(std::make_shared<MockApplication>())
      , mock_semantic_version_(utils::SemanticVersion(5, 1, 0)) {}

  ~RPCPassingHandlerTest() {}

  void SetUp() OVERRIDE {
    rpc_passing_handler_ =
        new am::RPCPassingHandler(mock_app_service_manager_, mock_app_manager_);
    app_services_.clear();

    ON_CALL(mock_app_manager_, IncreaseForwardedRequestTimeout(_, _))
        .WillByDefault(Return());
    ON_CALL(mock_app_manager_, get_settings())
        .WillByDefault(ReturnRef(mock_app_manager_settings_));

    ON_CALL(mock_app_manager_settings_, rpc_pass_through_timeout())
        .WillByDefault(Return(kRPCPassthroughTimeout));

    ON_CALL(mock_app_manager_, GetRPCService())
        .WillByDefault(ReturnRef(mock_rpc_service_));
    ON_CALL(mock_rpc_service_, ManageMobileCommand(_, _))
        .WillByDefault(Return(true));
    ON_CALL(mock_rpc_service_, SendMessageToMobile(_, _))
        .WillByDefault(Return());

    ON_CALL(mock_app_manager_, GetAppServiceManager())
        .WillByDefault(ReturnRef(mock_app_service_manager_));
    ON_CALL(mock_app_service_manager_, GetActiveServices())
        .WillByDefault(Return(app_services_));

    ON_CALL(mock_app_manager_, GetRPCHandler())
        .WillByDefault(ReturnRef(mock_rpc_handler_));
    ON_CALL(mock_rpc_handler_, ValidateRpcSO(_, _, _, _))
        .WillByDefault(Return(true));

    ON_CALL(mock_app_manager_, application(_))
        .WillByDefault(Return(mock_app_ptr_));
    ON_CALL(*mock_app_ptr_, msg_version())
        .WillByDefault(ReturnRef(mock_semantic_version_));
  }

  void TearDown() OVERRIDE {
    delete rpc_passing_handler_;
    rpc_passing_handler_ = NULL;
  }

 protected:
  am::AppService CreateAppService(uint32_t connection_key,
                                  const std::string& service_id,
                                  const std::string& service_type) {
    am::AppService app_service;

    app_service.connection_key = connection_key;
    app_service.mobile_service = true;
    app_service.default_service = false;

    smart_objects::SmartObject record(smart_objects::SmartType::SmartType_Map);
    record[am::strings::service_id] = service_id;
    // record[am::strings::service_manifest][am::strings::service_name] =
    // service_name;
    record[am::strings::service_manifest][am::strings::service_type] =
        service_type;
    record[am::strings::service_manifest][am::strings::allow_app_consumers] =
        true;
    record[am::strings::service_manifest][am::strings::handled_rpcs] =
        smart_objects::SmartObject(smart_objects::SmartType::SmartType_Array);
    record[am::strings::service_manifest][am::strings::handled_rpcs][0] =
        mobile_apis::FunctionID::SendLocationID;
    record[am::strings::service_published] = true;
    record[am::strings::service_active] = true;
    app_service.record = record;

    return app_service;
  }

  smart_objects::SmartObject CreatePassThroughRequest(uint32_t connection_key,
                                                      int32_t correlation_id) {
    smart_objects::SmartObject record(smart_objects::SmartType::SmartType_Map);
    record[am::strings::params][am::strings::connection_key] = connection_key;
    record[am::strings::params][am::strings::correlation_id] = correlation_id;
    record[am::strings::params][am::strings::function_id] =
        mobile_apis::FunctionID::SendLocationID;
    record[am::strings::params][am::strings::message_type] =
        am::MessageType::kRequest;
    record[am::strings::msg_params][am::strings::latitude_degrees] = 50;
    record[am::strings::msg_params][am::strings::longitude_degrees] = 50;
    record[am::strings::msg_params][am::strings::location_name] =
        "TestLocation";

    return record;
  }

  smart_objects::SmartObject CreatePassThroughResponse(
      uint32_t connection_key,
      int32_t correlation_id,
      const std::string& result_code,
      bool success,
      std::string info = std::string()) {
    smart_objects::SmartObject record(smart_objects::SmartType::SmartType_Map);
    record[am::strings::params][am::strings::connection_key] = connection_key;
    record[am::strings::params][am::strings::correlation_id] = correlation_id;
    record[am::strings::params][am::strings::function_id] =
        mobile_apis::FunctionID::SendLocationID;
    record[am::strings::params][am::strings::message_type] =
        am::MessageType::kResponse;
    record[am::strings::msg_params][am::strings::result_code] = result_code;
    record[am::strings::msg_params][am::strings::success] = success;
    record[am::strings::msg_params][am::strings::info] = info;

    return record;
  }

  void SendPassthroughRequestToMobile(uint32_t connection_key,
                                      int32_t correlation_id) {
    smart_objects::SmartObject request =
        CreatePassThroughRequest(connection_key, correlation_id);
    smart_objects::SmartObject forwarded_request =
        CreatePassThroughRequest(connection_key + 1, correlation_id);

    {
      InSequence dummy;
      // Call RPCPassThrough with request smart object

      // Will call PopulateRPCRequestQueue
      EXPECT_CALL(mock_app_service_manager_, GetActiveServices())
          .WillOnce(Return(app_services_));
      EXPECT_CALL(
          mock_app_manager_,
          IncreaseForwardedRequestTimeout(connection_key, correlation_id))
          .Times(app_services_.size());

      // Will call ForwardRequesttoMobile
      EXPECT_CALL(mock_app_manager_, get_settings());
      EXPECT_CALL(mock_app_manager_settings_, rpc_pass_through_timeout());
      EXPECT_CALL(mock_app_manager_, GetRPCService());
      EXPECT_CALL(mock_rpc_service_,
                  SendMessageToMobile(Pointee(forwarded_request), false));
    }

    bool result = rpc_passing_handler_->RPCPassThrough(request);
    EXPECT_EQ(result, true);
  }

  void SendPassthroughResponseFromMobile(uint32_t connection_key,
                                         int32_t correlation_id) {}

  // void (uint32_t connection_key, int32_t correlation_id){}

  MockApplicationManager mock_app_manager_;
  MockApplicationManagerSettings mock_app_manager_settings_;
  MockRPCService mock_rpc_service_;
  MockRPCHandler mock_rpc_handler_;
  resumption_test::MockLastState mock_last_state_;
  MockAppServiceManager mock_app_service_manager_;
  std::shared_ptr<MockApplication> mock_app_ptr_;
  const utils::SemanticVersion mock_semantic_version_;
  am::RPCPassingHandler* rpc_passing_handler_;
  std::vector<am::AppService> app_services_;

  uint32_t kRPCPassthroughTimeout = 10000;
};

TEST_F(RPCPassingHandlerTest, RPCPassingTest_REQUEST_ForwardToMobile) {
  int32_t correlation_id = 1;
  uint32_t connection_key = 1;
  app_services_.push_back(
      CreateAppService(connection_key + 1, "service 1", "NAVIGATION"));

  SendPassthroughRequestToMobile(connection_key, correlation_id);
}

TEST_F(RPCPassingHandlerTest, RPCPassingTest_REQUEST_NoPassthrough) {
  int32_t correlation_id = 1;
  uint32_t connection_key = 1;

  smart_objects::SmartObject request =
      CreatePassThroughRequest(connection_key, correlation_id);

  {
    InSequence dummy;
    // Call RPCPassThrough with request smart object

    // Will call PopulateRPCRequestQueue
    EXPECT_CALL(mock_app_service_manager_, GetActiveServices())
        .WillOnce(Return(app_services_));
    EXPECT_CALL(mock_app_manager_,
                IncreaseForwardedRequestTimeout(connection_key, correlation_id))
        .Times(0);
    // Will return false since there are no active services to handle the rpc
    EXPECT_CALL(mock_rpc_service_, ManageMobileCommand(_, _)).Times(0);
    EXPECT_CALL(mock_rpc_service_, SendMessageToMobile(_, _)).Times(0);
  }

  bool result = rpc_passing_handler_->RPCPassThrough(request);
  EXPECT_EQ(result, false);
}

TEST_F(RPCPassingHandlerTest, RPCPassingTest_RESPONSE_UnknownCorrelationID) {
  int32_t correlation_id = 1;
  uint32_t connection_key = 1;
  smart_objects::SmartObject response = CreatePassThroughResponse(
      connection_key, correlation_id, "SUCCESS", true);

  app_services_.push_back(
      CreateAppService(connection_key + 1, "service 1", "NAVIGATION"));

  {
    InSequence dummy;
    // Call RPCPassThrough with response smart object
    // Will return false since the correlation id does not exist in the map
    EXPECT_CALL(mock_rpc_service_, ManageMobileCommand(_, _)).Times(0);
    EXPECT_CALL(mock_rpc_service_, SendMessageToMobile(_, _)).Times(0);
  }
  bool result = rpc_passing_handler_->RPCPassThrough(response);
  EXPECT_EQ(result, false);
}

TEST_F(RPCPassingHandlerTest, RPCPassingTest_SUCCESS) {
  int32_t correlation_id = 1;
  uint32_t connection_key = 1;
  smart_objects::SmartObject response = CreatePassThroughResponse(
      connection_key + 1, correlation_id, "SUCCESS", true);
  smart_objects::SmartObject forwarded_response = CreatePassThroughResponse(
      connection_key, correlation_id, "SUCCESS", true);

  app_services_.push_back(
      CreateAppService(connection_key + 1, "service 1", "NAVIGATION"));

  SendPassthroughRequestToMobile(connection_key, correlation_id);

  {
    InSequence dummy;
    // Call RPCPassThrough with response smart object
    // Will ForwardResponseToMobile
    EXPECT_CALL(mock_app_manager_, GetRPCService());
    EXPECT_CALL(mock_rpc_service_,
                SendMessageToMobile(Pointee(forwarded_response), false));
  }
  bool result = rpc_passing_handler_->RPCPassThrough(response);
  EXPECT_EQ(result, true);
}

TEST_F(RPCPassingHandlerTest,
       RPCPassingTest_UNSUPPORTED_REQUEST_ForwardToCore) {
  int32_t correlation_id = 1;
  uint32_t connection_key = 1;
  smart_objects::SmartObject unsupported_response = CreatePassThroughResponse(
      connection_key + 1, correlation_id, "UNSUPPORTED_REQUEST", false);
  smart_objects::SmartObject forwarded_request =
      CreatePassThroughRequest(connection_key, correlation_id);

  app_services_.push_back(
      CreateAppService(connection_key + 1, "service 1", "NAVIGATION"));

  SendPassthroughRequestToMobile(connection_key, correlation_id);

  {
    InSequence dummy;
    // Call RPCPassThrough with response smart object
    // Will cycle to core (no other app services in list)
    EXPECT_CALL(mock_app_manager_, application(connection_key));
    EXPECT_CALL(*mock_app_ptr_, msg_version());
    EXPECT_CALL(mock_app_manager_, GetRPCHandler());
    EXPECT_CALL(mock_rpc_handler_,
                ValidateRpcSO(forwarded_request, _, _, false))
        .WillOnce(Return(true));
    EXPECT_CALL(mock_app_manager_, GetRPCService());
    EXPECT_CALL(mock_rpc_service_,
                ManageMobileCommand(Pointee(forwarded_request),
                                    am::commands::Command::SOURCE_MOBILE));
  }
  bool result = rpc_passing_handler_->RPCPassThrough(unsupported_response);
  EXPECT_EQ(result, true);
}

TEST_F(RPCPassingHandlerTest,
       RPCPassingTest_UNSUPPORTED_REQUEST_ForwardToMobile) {
  int32_t correlation_id = 1;
  uint32_t connection_key = 1;
  smart_objects::SmartObject unsupported_response = CreatePassThroughResponse(
      connection_key + 1, correlation_id, "UNSUPPORTED_REQUEST", false);
  smart_objects::SmartObject forwarded_request =
      CreatePassThroughRequest(connection_key + 2, correlation_id);

  app_services_.push_back(
      CreateAppService(connection_key + 1, "service 1", "NAVIGATION"));
  app_services_.push_back(
      CreateAppService(connection_key + 2, "service 2", "MEDIA"));

  SendPassthroughRequestToMobile(connection_key, correlation_id);

  {
    InSequence dummy;
    // Call RPCPassThrough with response smart object
    // Will cycle to core (no other app services in list)
    EXPECT_CALL(mock_app_manager_, get_settings());
    EXPECT_CALL(mock_app_manager_settings_, rpc_pass_through_timeout());
    EXPECT_CALL(mock_app_manager_, GetRPCService());
    EXPECT_CALL(mock_rpc_service_,
                SendMessageToMobile(Pointee(forwarded_request), false));
  }
  bool result = rpc_passing_handler_->RPCPassThrough(unsupported_response);
  EXPECT_EQ(result, true);
}

}  // namespace application_manager_test
}  // namespace components
}  // namespace test
