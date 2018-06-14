/*
 * Copyright (c) 2018, Ford Motor Company
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

#ifndef SRC_APPMAIN_LOW_VOLTAGE_SIGNALS_HANDLER_H_
#define SRC_APPMAIN_LOW_VOLTAGE_SIGNALS_HANDLER_H_

#include <unistd.h>
#include "utils/threads/thread_delegate.h"
#include "utils/threads/thread.h"

namespace main_namespace {

typedef struct LowVoltageSignalsOffset {
  int low_voltage_signal_offset;
  int wake_up_signal_offset;
  int ignition_off_signal_offset;
} LowVoltageSignalsOffset;

class LifeCycle;
class NotificationThreadDelegate;

/**
 * @brief The SDLState enum defines current Smartdevicelink app state
 * which can be changed due to received UNIX RT signals dedicated for
 * Low Voltage functionality handling
 * e.g. LowVoltage, WakeUp, IgnitionOff signals
 */
enum class SDLState { kRun, kSleep, kStop };

class LowVoltageSignalsHandler {
 public:

  /**
   * @brief Constructor
   */
  LowVoltageSignalsHandler(LifeCycle& life_cycle,
                           const LowVoltageSignalsOffset& offset_data);
  /**
   * @brief Creates all parts of LowVoltageSignalsHandler
   */
  bool Init();
  void HandleSignal(const int signo);
  bool IsActive() const;
  SDLState get_current_sdl_state() const;
  int low_voltage_signo() const;
  int wake_up_signo() const;
  int ignition_off_signo() const;
  ~LowVoltageSignalsHandler();

 private:
  void Destroy();
  SDLState state_;
  NotificationThreadDelegate* notifications_delegate_;
  LifeCycle& life_cycle_;
  threads::Thread* signals_handler_thread_;
  int SIGLOWVOLTAGE_;
  int SIGWAKEUP_;
  int SIGIGNOFF_;
};

class NotificationThreadDelegate : public threads::ThreadDelegate {
 public:
  NotificationThreadDelegate(
      LowVoltageSignalsHandler& low_voltage_signals_handler)
      : low_voltage_signals_handler_(low_voltage_signals_handler) {}

  ~NotificationThreadDelegate() {}

  void threadMain() OVERRIDE;
  void exitThreadMain() OVERRIDE;

 private:
  LowVoltageSignalsHandler& low_voltage_signals_handler_;
};

}  // namespace main_namespace

#endif  // SRC_APPMAIN_LOW_VOLTAGE_SIGNALS_HANDLER_H_
