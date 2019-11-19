/*
 * \file transport_manager_default.h
 * \brief Transport manager default class header file.
 *
 * Copyright (c) 2013, Ford Motor Company
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

#ifndef SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_TRANSPORT_MANAGER_DEFAULT_H_
#define SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_TRANSPORT_MANAGER_DEFAULT_H_

#include "transport_manager/transport_adapter/transport_adapter.h"
#include "transport_manager/transport_manager_impl.h"

namespace resumption {
class LastState;
}

namespace transport_manager {

/**
 * @brief Default realization of transport_manager_impl class.
 */
class TransportManagerDefault : public TransportManagerImpl {
 public:
  explicit TransportManagerDefault(const TransportManagerSettings& settings);

  /**
   * @brief Initialize transport manager.
   *
   * @return Code error.
   */
  int Init(resumption::LastState& last_state) OVERRIDE;

  /**
   * @brief Destructor.
   */
  virtual ~TransportManagerDefault();

#if defined(BUILD_TESTS)
  void set_ta_bluetooth(transport_adapter::TransportAdapter* ta_bluetooth);
  void set_ta_tcp(transport_adapter::TransportAdapter* ta_tcp);
  void set_ta_usb(transport_adapter::TransportAdapter* ta_usb);
  void set_ta_cloud(transport_adapter::TransportAdapter* ta_cloud);
#endif  // BUILD_TESTS

 private:
  transport_adapter::TransportAdapter* ta_bluetooth_;
  transport_adapter::TransportAdapter* ta_tcp_;
  transport_adapter::TransportAdapter* ta_usb_;
  transport_adapter::TransportAdapter* ta_cloud_;
  DISALLOW_COPY_AND_ASSIGN(TransportManagerDefault);
};
}  // namespace transport_manager

#endif  // SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_TRANSPORT_MANAGER_DEFAULT_H_
