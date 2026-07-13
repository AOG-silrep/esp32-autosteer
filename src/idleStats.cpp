// MIT License
//
// Copyright (c) 2020 Christian Riggenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <ESPUI.h>
#include "esp_freertos_hooks.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "soc/soc.h"
#include "soc/timer_group_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/rtc_cntl_reg.h"

#include "main.hpp"

volatile uint16_t idleCtrCore0 = 0;
volatile uint16_t idleCtrCore1 = 0;

volatile uint32_t rtcRefCpuMHzCurrent = 0;
volatile uint32_t rtcRefCpuMHzHigh = 0;
volatile uint32_t rtcRefCpuMHzLow = UINT32_MAX;

constexpr uint32_t rtcRefCpuMHzWindowSamples = 300;  // 5 minutes at the 1s sample interval below
uint16_t rtcRefCpuMHzWindow[rtcRefCpuMHzWindowSamples];
uint32_t rtcRefCpuMHzWindowIdx = 0;
uint32_t rtcRefCpuMHzWindowCount = 0;

uint32_t xtalFreqMHz = 40;

bool core0IdleWorker( void ) {
  static TickType_t xLastWakeTime0;

  if( xLastWakeTime0 != xTaskGetTickCount() ) {
    xLastWakeTime0 = xTaskGetTickCount();
    idleCtrCore0++;
  }

  return true;
}

bool core1IdleWorker( void ) {
  static TickType_t xLastWakeTime1;

  if( xLastWakeTime1 != xTaskGetTickCount() ) {
    xLastWakeTime1 = xTaskGetTickCount();
    idleCtrCore1++;
  }

  return true;
}

uint32_t IRAM_ATTR rtcCalSampleCpuMHz( uint32_t slowclkCycles, uint32_t xtalMHz ) {
  REG_SET_FIELD( TIMG_RTCCALICFG_REG( 0 ), TIMG_RTC_CALI_CLK_SEL, RTC_CAL_8MD256 );
  CLEAR_PERI_REG_MASK( TIMG_RTCCALICFG_REG( 0 ), TIMG_RTC_CALI_START_CYCLING );
  REG_SET_FIELD( TIMG_RTCCALICFG_REG( 0 ), TIMG_RTC_CALI_MAX, slowclkCycles );
  CLEAR_PERI_REG_MASK( TIMG_RTCCALICFG_REG( 0 ), TIMG_RTC_CALI_START );

  constexpr uint32_t maxPollCycles = 10 * 1000 * 1000;  // generous timeout, tolerant of preemption

  uint32_t preCycles = ESP.getCycleCount();
  SET_PERI_REG_MASK( TIMG_RTCCALICFG_REG( 0 ), TIMG_RTC_CALI_START );

  // Phase 1: wait for RDY to clear, confirming the fresh START was acknowledged and any stale
  // RDY=1 left over from the previous call (not yet cleared across the slow-clock synchronizer)
  // is gone. A fixed guessed delay here would bias postCycles later than the true completion.
  while( GET_PERI_REG_MASK( TIMG_RTCCALICFG_REG( 0 ), TIMG_RTC_CALI_RDY ) ) {
    if( ( ESP.getCycleCount() - preCycles ) > maxPollCycles ) {
      return 0;
    }
  }

  // Phase 2: tightly poll for the new calibration to complete.
  while( !GET_PERI_REG_MASK( TIMG_RTCCALICFG_REG( 0 ), TIMG_RTC_CALI_RDY ) ) {
    if( ( ESP.getCycleCount() - preCycles ) > maxPollCycles ) {
      return 0;
    }
  }

  uint32_t postCycles = ESP.getCycleCount();
  uint32_t xtalCycles = REG_GET_FIELD( TIMG_RTCCALICFG1_REG( 0 ), TIMG_RTC_CALI_VALUE );

  if( xtalCycles == 0 ) {
    return 0;
  }

  double totalElapsedUs = ( double ) xtalCycles / xtalMHz;
  return ( uint32_t )( ( postCycles - preCycles ) / totalElapsedUs );
}

void rtcRefCpuCheckWorker( void* z ) {
  constexpr TickType_t xFrequency = 1000;  // 1s sample interval
  constexpr uint32_t rtcCalCycles = 100;   // ~1.5-3ms busy-wait per sample, fine at priority 1
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while( 1 ) {
    uint32_t mhz = rtcCalSampleCpuMHz( rtcCalCycles, xtalFreqMHz );

    if( mhz != 0 ) {
      rtcRefCpuMHzCurrent = mhz;

      rtcRefCpuMHzWindow[rtcRefCpuMHzWindowIdx] = ( uint16_t ) mhz;
      rtcRefCpuMHzWindowIdx = ( rtcRefCpuMHzWindowIdx + 1 ) % rtcRefCpuMHzWindowSamples;

      if( rtcRefCpuMHzWindowCount < rtcRefCpuMHzWindowSamples ) { rtcRefCpuMHzWindowCount++; }

      // Recompute over the trailing window each sample so Hi/Lo are a true rolling 5-minute
      // range rather than a periodic reset; 300 comparisons at 1Hz is negligible on this chip.
      uint32_t high = 0;
      uint32_t low = UINT32_MAX;

      for( uint32_t i = 0; i < rtcRefCpuMHzWindowCount; i++ ) {
        uint16_t v = rtcRefCpuMHzWindow[i];

        if( v > high ) { high = v; }

        if( v < low )  { low  = v; }
      }

      rtcRefCpuMHzHigh = high;
      rtcRefCpuMHzLow  = low;
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void idleStatsWorker( void* z ) {
  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  String str;
  str.reserve( 500 );

  while( 1 ) {
    str = "<a href='/diagnostics' style='color:inherit;text-decoration:none;display:block;'>Core0: ";
    str += ( 1000 - idleCtrCore0 ) / 10;
    str += "‰<br/>";
    str += "Core1: ";
    str += ( 1000 - idleCtrCore1 ) / 10;
    str += "‰<br/>CPU(RTC-ref): ";
    str += rtcRefCpuMHzCurrent;
    str += "MHz 5m Hi:";
    str += rtcRefCpuMHzHigh;
    str += " Lo:";
    str += rtcRefCpuMHzLow;
    str += "<br/>Uptime: ";
    str += millis() / 1000;
    str += "s<br/>RSSI: ";
    str += WiFi.RSSI();
    str += "dBm<br/>";

    if ( xTimerIsTimerActive( saveTimer ) ) {
      TickType_t remaining = xTimerGetExpiryTime( saveTimer ) - xTaskGetTickCount();
      str += "Saving in ";
      str += ( remaining / configTICK_RATE_HZ ) + 1;
      str += "s...";
    } else {
      multi_heap_info_t heapInfo;
      heap_caps_get_info( &heapInfo, MALLOC_CAP_8BIT );
      str += "Heap free: ";
      str += heapInfo.total_free_bytes / 1024;
      str += "kB (";
      str += heapInfo.free_blocks;
      str += "), allocated: ";
      str += heapInfo.total_allocated_bytes / 1024;
      str += "kB (";
      str += heapInfo.allocated_blocks;
      str += ")<br/>Lowest ever free Heap: ";
      str += heapInfo.minimum_free_bytes / 1024;
      str += "kB<br/>Largest free block on Heap: ";
      str += heapInfo.largest_free_block / 1024;
      str += "kB";
    }

    str += "</a>";

    Control* labelLoadHandle = ESPUI.getControl( labelLoad );
    labelLoadHandle->value = str;
    ESPUI.updateLabel( labelLoad, str );

    idleCtrCore0 = 0;
    idleCtrCore1 = 0;

//   heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initIdleStats() {
  esp_register_freertos_idle_hook_for_cpu( core0IdleWorker, 0 );
  esp_register_freertos_idle_hook_for_cpu( core1IdleWorker, 1 );
  rtc_clk_8m_enable( true, true );
  SET_PERI_REG_MASK( RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_CLK8M_D256_EN_M );  // bridge the 8MHz/256 RTC clock into the digital domain so TIMG0 can see it
  periph_module_enable( PERIPH_TIMG0_MODULE );  // TIMG0 hosts the RTC calibration registers used by rtcCalSampleCpuMHz()
  SET_PERI_REG_MASK( TIMGCLK_REG( 0 ), TIMG_CLK_EN );  // force-enable TIMG0's internal regfile clock, off by default
  xtalFreqMHz = ( uint32_t ) rtc_clk_xtal_freq_get();
  xTaskCreate( idleStatsWorker, "IdleStats", 2048, NULL, 10, NULL );
  xTaskCreate( rtcRefCpuCheckWorker, "RtcRefCpuCheck", 2048, NULL, 1, NULL );
}
