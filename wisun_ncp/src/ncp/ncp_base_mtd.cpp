/*
 *    Copyright (c) 2016-2017, The OpenThread Authors.
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 *    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /**
  * @file
  *   This file implements minimal thread device required Spinel interface to the OpenThread stack.
  */

#include "openthread-core-config.h"
#include "ncp_base.hpp"

#if OPENTHREAD_CONFIG_BORDER_ROUTER_ENABLE
#include <openthread/border_router.h>
#endif
#if OPENTHREAD_CONFIG_CHANNEL_MONITOR_ENABLE
#include <openthread/channel_monitor.h>
#endif
#if OPENTHREAD_CONFIG_CHILD_SUPERVISION_ENABLE
#include <openthread/child_supervision.h>
#endif
#include <openthread/diag.h>
#include <openthread/icmp6.h>
#if OPENTHREAD_CONFIG_JAM_DETECTION_ENABLE
#include <openthread/jam_detection.h>
#endif
#include <openthread/ncp.h>
#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
#include <openthread/network_time.h>
#endif
#include <openthread/platform/misc.h>
#include <openthread/platform/radio.h>
#if OPENTHREAD_FTD
#include <openthread/thread_ftd.h>
#endif
#if OPENTHREAD_CONFIG_TMF_NETDATA_SERVICE_ENABLE
#include <openthread/server.h>
#endif

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/instance.hpp"
  //#include "net/ip6.hpp"

#include <string.h>

#include "otstack.h"
#include <ti/drivers/GPIO.h>
//#include <ti_wisunfan_config.h>
//#include "ti_drivers_config.h"
//Additional header files for integrating with nanostack
#include "nsconfig.h"
#include "Core/include/ns_buffer.h"
#include "ns_trace.h"
#include <openthread/message.h>
#include "ncp_interface/src/core/common/message.hpp"
#include "common/locator.hpp"
#include "Common_Protocols/ipv6_constants.h"

#include "mbed-mesh-api/mesh_interface_types.h"
//#include "api_mac.h"
#include "mac_spec.h"
#include "saddr.h"
#include "application.h"
#include "Core/include/ns_address_internal.h"
#include <ioc.h>
#include "ti_radio_config.h"

#define WISUN_PROTOCOL_VERSION_MAJOR 1
#define WISUN_PROTOCOL_VERSION_MINOR 0
#define WISUN_PROTOCOL_VERSION_SECOND_MINOR 0

#define INTERFACE_TYPE_WISUN 4

#define STACK_NAME "TIWISUNFAN"
#define STACK_VERSION "1.0.0"
#define BUILD_INFO
#define OTHER_INFO "RELEASE"
#define TRACE_GROUP "ncp"

#define PROTOCOL_NAME "Wi-SUNFAN"
#define PROTOCOL_VERSION "1.0"

namespace ot {
    namespace Ncp {

#ifdef WISUN_NCP_ENABLE

        extern "C" configurable_props_t cfg_props;

#ifdef SWITCH_NCP_TO_TRACE
        extern "C" uint32_t g_switchNcp2Trace;
#endif //SWITCH_NCP_TO_TRACE



        /* Core properties */

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_LAST_STATUS>(void) {
            return mEncoder.WriteUintPacked(mLastStatus);
        }

        const char* GetProtocolVersionString(void) {
            static const char sVersion[] = PROTOCOL_NAME "/" PROTOCOL_VERSION
                ; // Trailing semicolon to end statement.

            return sVersion;
        }

        const char* GetVersionString(void) {

            static const char sVersion[] = STACK_NAME "/" STACK_VERSION "; " OTHER_INFO
#if defined(__DATE__)
                "; " __DATE__ " " __TIME__
#endif
                ; // Trailing semicolon to end statement.

            return sVersion;
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PROTOCOL_VERSION>(void) {

            //return mEncoder.WriteUtf8(GetProtocolVersionString());
            return mEncoder.WriteUtf8("I run this city");

        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_DODAG_ROUTE>(void) {
            mEncoder.WriteUint8(69);
           return mEncoder.WriteUint8(70);
        }


        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_NCP_VERSION>(void) {
            return mEncoder.WriteUtf8(GetVersionString());
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_INTERFACE_TYPE>(void) {
            return mEncoder.WriteUintPacked(INTERFACE_TYPE_WISUN);
        }

        extern "C" void ccfg_read_mac_addr(uint8_t * mac_addr);
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_HWADDR>(void) {
            uint8_t hwAddr[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

            ccfg_read_mac_addr(hwAddr);

            return mEncoder.WriteEui64(hwAddr);
        }
        /* PHY properties */


        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_CCA_THRESHOLD>(void) {
            return mEncoder.WriteInt8(cfg_props.ccaDefaultdBm);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_PHY_CCA_THRESHOLD>(void) {
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadInt8(cfg_props.ccaDefaultdBm));

        exit:
            return error;
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_TX_POWER>(void) {
            return mEncoder.WriteInt8(cfg_props.phyTxPower);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_PHY_TX_POWER>(void) {
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadInt8(cfg_props.phyTxPower));

        exit:
            return error;
        }


        /* MAC properties */
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_MAC_15_4_PANID>(void) {
            return mEncoder.WriteUint16(cfg_props.pan_id);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_MAC_15_4_PANID>(void) {
            otError  error = OT_ERROR_NONE;
            SuccessOrExit(error = mDecoder.ReadUint16(cfg_props.pan_id));

        exit:
            return error;
        }

        /* NET properties */
        extern "C" bool is_net_if_up();
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_NET_IF_UP>(void) {
            return mEncoder.WriteBool((bool)is_net_if_up());
        }

        extern "C" otError nanostack_net_if_up();
        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_NET_IF_UP>(void) {
            bool    enable = false;
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadBool(enable));

            //bring interface up if currently the interface is down
            if (true == enable)     {
                if (!is_net_if_up())         {
                    error = nanostack_net_if_up();
                }
            }
            //to be done: enable = false case

        exit:
            return error;
        }

        extern "C" bool is_net_stack_up();
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_NET_STACK_UP>(void) {
            return mEncoder.WriteBool((bool)is_net_stack_up());
        }

        extern "C" otError nanostack_net_stack_up(void);
        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_NET_STACK_UP>(void) {
            bool    enable = false;
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadBool(enable));

            if (true == enable)     {
                //if net stack not already brought up
                if (!is_net_stack_up()) 		{

#ifdef SWITCH_NCP_TO_TRACE
                    //set a flag to drop pushing info from NCP onto Primary Uart
                    g_switchNcp2Trace = 1;

                    //reassign the uart pin to trace
#if defined(LP_CC1312R7) || defined(LAUNCHXL_CC1312R1)
                    IOCPortConfigureSet(IOID_3, IOC_PORT_MCU_SWV, IOC_STD_OUTPUT);
#else  // 1312
                    IOCPortConfigureSet(IOID_13, IOC_PORT_MCU_SWV, IOC_STD_OUTPUT);
#endif // 1312
#endif // SWITCH_NCP_TO_TRACE              

                    //bringup the wisunstack
                    error = nanostack_net_stack_up();
                }
            }//(true == enabled)

        exit:
            return error;
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_NET_ROLE>(void) {
            bool netRole = 0; // 0 - Border Router; 1 - Router
            if (cfg_props.wisun_device_type == MESH_DEVICE_TYPE_WISUN_BORDER_ROUTER)     {
                netRole = 0;
            }
            else     {
                netRole = 1;
            }
            return mEncoder.WriteBool((bool)netRole);
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_NET_NETWORK_NAME>(void) {
            return mEncoder.WriteUtf8((char*)cfg_props.network_name);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_NET_NETWORK_NAME>(void) {
            const char* string = NULL;
            otError     error = OT_ERROR_NONE;

            SuccessOrExit(mDecoder.ReadUtf8(string));

            memset(cfg_props.network_name, 0, MAX_NETWORK_NAME_SIZE);
            strcpy((char*)cfg_props.network_name, string);

        exit:
            return error;
        }

        /* Tech Specific: PHY properties */
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_REGION>(void) {
            return mEncoder.WriteUint8(cfg_props.config_reg_domain);
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_MODE_ID>(void) {
            return mEncoder.WriteUint8(cfg_props.config_phy_id);
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_UNICAST_CHANNEL_LIST>(void) {
            otError error = OT_ERROR_NONE;
            uint8_t i = 0;

            for (i = 0; i < CHANNEL_BITMAP_SIZE; i++)     {
                SuccessOrExit(error = mEncoder.WriteUint8(cfg_props.uc_channel_list[i]));
            }

        exit:
            return(error);
        }

        extern "C" uint8_t get_first_fixed_channel(uint8_t * channel_list);
        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_PHY_UNICAST_CHANNEL_LIST>(void) {
            otError error = OT_ERROR_NONE;
            uint8_t i = 0;

            const uint8_t* chListPtr = NULL;
            uint16_t       chListLen = 0;

            SuccessOrExit(error = mDecoder.ReadDataWithLen(chListPtr, chListLen));

            for (i = 0; i < CHANNEL_BITMAP_SIZE; i++)    {
                cfg_props.uc_channel_list[i] = chListPtr[i];
            }

            // Update fixed unicast channel based on new selection
            cfg_props.uc_fixed_channel = get_first_fixed_channel(cfg_props.uc_channel_list);

        exit:
            return(error);
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_BROADCAST_CHANNEL_LIST>(void) {
            otError error = OT_ERROR_NONE;
            uint8_t i = 0;

            for (i = 0; i < CHANNEL_BITMAP_SIZE; i++)     {
                SuccessOrExit(error = mEncoder.WriteUint8(cfg_props.bc_channel_list[i]));
            }

        exit:
            return(error);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_PHY_BROADCAST_CHANNEL_LIST>(void) {
            otError error = OT_ERROR_NONE;
            uint8_t i = 0;

            const uint8_t* chListPtr = NULL;
            uint16_t       chListLen = 0;

            SuccessOrExit(error = mDecoder.ReadDataWithLen(chListPtr, chListLen));

            for (i = 0; i < CHANNEL_BITMAP_SIZE; i++)    {
                cfg_props.bc_channel_list[i] = chListPtr[i];
            }

            // Update fixed broadcast channel based on new selection
            cfg_props.bc_fixed_channel = get_first_fixed_channel(cfg_props.bc_channel_list);

        exit:
            return(error);
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_ASYNC_CHANNEL_LIST>(void) {
            otError error = OT_ERROR_NONE;
            uint8_t i = 0;

            for (i = 0; i < CHANNEL_BITMAP_SIZE; i++)     {
                SuccessOrExit(error = mEncoder.WriteUint8(cfg_props.async_channel_list[i]));
            }

        exit:
            return(error);

        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_PHY_ASYNC_CHANNEL_LIST>(void) {
            otError error = OT_ERROR_NONE;
            uint8_t i = 0;

            const uint8_t* chListPtr = NULL;
            uint16_t       chListLen = 0;

            SuccessOrExit(error = mDecoder.ReadDataWithLen(chListPtr, chListLen));

            for (i = 0; i < CHANNEL_BITMAP_SIZE; i++)    {
                cfg_props.async_channel_list[i] = chListPtr[i];
            }

        exit:
            return(error);
        }


        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_CH_SPACING>(void) {
            return mEncoder.WriteUint16(cfg_props.config_channel_spacing);
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_PHY_CHO_CENTER_FREQ>(void) {
            otError     error = OT_ERROR_NONE;

            uint16_t Ch0freq_MHz = (uint16_t)(cfg_props.ch0_center_frequency / 1000);
            uint16_t Ch0freq_kHz = (uint16_t)(cfg_props.ch0_center_frequency - Ch0freq_MHz * 1000);

            SuccessOrExit(error = mEncoder.WriteUint16(Ch0freq_MHz));
            SuccessOrExit(error = mEncoder.WriteUint16(Ch0freq_kHz));

        exit:
            return error;
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_MAC_BC_INTERVAL>(void) {
            return mEncoder.WriteUint32(cfg_props.bc_interval);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_MAC_BC_INTERVAL>(void) {
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadUint32(cfg_props.bc_interval));

        exit:
            return error;
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_MAC_BC_DWELL_INTERVAL>(void) {
            return mEncoder.WriteUint8(cfg_props.bc_dwell_interval);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_MAC_BC_DWELL_INTERVAL>(void) {
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadUint8(cfg_props.bc_dwell_interval));

        exit:
            return error;
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_MAC_BC_CHANNEL_FUNCTION>(void) {
            return mEncoder.WriteUint8(cfg_props.bc_channel_function);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_MAC_BC_CHANNEL_FUNCTION>(void) {
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadUint8(cfg_props.bc_channel_function));

        exit:
            return error;
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_MAC_UC_DWELL_INTERVAL>(void) {
            return mEncoder.WriteUint8(cfg_props.uc_dwell_interval);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_MAC_UC_DWELL_INTERVAL>(void) {
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadUint8(cfg_props.uc_dwell_interval));

        exit:
            return error;
        }

        template <> otError NcpBase::HandlePropertyGet<SPINEL_MAC_UC_CHANNEL_FUNCTION>(void) {
            return mEncoder.WriteUint8(cfg_props.uc_channel_function);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_MAC_UC_CHANNEL_FUNCTION>(void) {
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadUint8(cfg_props.uc_channel_function));

        exit:
            return error;
        }

#ifdef TI_WISUN_FAN_DEBUG
        extern "C" uint8_t filterMode;
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_MAC_FILTER_MODE>(void) {
            return mEncoder.WriteUint8(filterMode);
        }

        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_MAC_FILTER_MODE>(void) {
            otError error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadUint8(filterMode));

        exit:
            return error;
        }
#endif //TI_WISUN_FAN_DEBUG

        extern "C" uint8_t * bitcopy(uint8_t * restrict dst, const uint8_t * restrict src, uint_fast8_t bits);
        extern "C" void nanostack_process_routing_table_update_from_stack(uint8_t changed_info, uint8_t * prefix, uint8_t len_prefix, uint8_t * addr_nexthop, uint32_t lifetime) {
            //make 16 byte IPv6 address from prefix
            uint8_t addr_self[16] = { 0 };
            bitcopy(addr_self, prefix, len_prefix);

            ot::Ncp::NcpBase* ncp = ot::Ncp::NcpBase::GetNcpInstance();
            ncp->SendRouteTableUpdate(changed_info, addr_self, len_prefix, addr_nexthop, lifetime);

            //what if an error occurs? when does the ncp try to send the failed update? How is the malformed spinel frame removed
        }

        otError NcpBase::SendRouteTableUpdate(uint8_t changed_info, uint8_t* addr_self, uint8_t len_prefix, uint8_t* addr_nexthop, uint32_t lifetime) {
            otError error = OT_ERROR_NONE;
            uint8_t           header = SPINEL_HEADER_FLAG | SPINEL_HEADER_IID_0;
            spinel_prop_key_t propKey = SPINEL_PROP_ROUTING_TABLE_UPDATE;

            // begin spinel encoding
            SuccessOrExit(error = mEncoder.BeginFrame(header, SPINEL_CMD_PROP_VALUE_IS, propKey));
            SuccessOrExit(error = mEncoder.OpenStruct());
            SuccessOrExit(error = mEncoder.WriteUint8(changed_info));
            SuccessOrExit(error = mEncoder.WriteIp6Address(addr_self));
            SuccessOrExit(error = mEncoder.WriteUint8(len_prefix));
            SuccessOrExit(error = mEncoder.WriteIp6Address(addr_nexthop));
            SuccessOrExit(error = mEncoder.WriteUint32(lifetime));
            SuccessOrExit(error = mEncoder.CloseStruct());
            SuccessOrExit(error = mEncoder.EndFrame());

        exit:
            return error;
        }


        extern "C" otError nanostack_process_stream_net_from_host(uint8_t * framePtr, uint16_t length);

        template <> otError NcpBase::HandlePropertySet<SPINEL_PROP_STREAM_NET>(void) {
            const uint8_t* framePtr = NULL;
            uint16_t       frameLen = 0;
            const uint8_t* metaPtr = NULL;
            uint16_t       metaLen = 0;
            otMessage* message = NULL;
            otError        error = OT_ERROR_NONE;

            SuccessOrExit(error = mDecoder.ReadDataWithLen(framePtr, frameLen));
            SuccessOrExit(error = mDecoder.ReadData(metaPtr, metaLen));

            nanostack_process_stream_net_from_host((uint8_t*)framePtr, frameLen);

        exit:
            mDroppedInboundIpFrameCounter++;
            return error;
        }

        void NcpBase::HandleDatagramFromStack(otMessage* aMessage) {
            VerifyOrExit(aMessage != NULL, OT_NOOP);

            SuccessOrExit(otMessageQueueEnqueue(&mMessageQueue, aMessage));

            // If there is no queued spinel command response, try to write/send
            // the datagram message immediately. If there is a queued response
            // or if currently out of buffer space, the IPv6 datagram message
            // will be sent from `HandleFrameRemovedFromNcpBuffer()` when buffer
            //  space becomes available and after any pending spinel command
            // response.

            if (IsResponseQueueEmpty())     {
                IgnoreReturnValue(SendQueuedDatagramMessages());
            }

        exit:
            return;
        }


        extern "C" void nanostack_process_stream_net_from_stack(buffer_t * buf) {
            ot::Ncp::NcpBase* ncp = ot::Ncp::NcpBase::GetNcpInstance();
            otMessageSettings settings = { true, OT_MESSAGE_PRIORITY_NORMAL };
            Message* message = NULL;
            uint8_t* aData = buffer_ipv6_pointer(buf);
            uint16_t aDataLength = buffer_ipv6_length(buf);

            if (ncp == NULL || buf->ipv6_buf_ptr == 0xFFFF)     {
                // tr_debug("\n NCP Instance is empty or IPv6 Buf Ptr is invalid!!!");
                goto exit;
            }

            VerifyOrExit((message = ncp->GetOtInstance()->mMessagePool.New(Message::kTypeIp6, 0, &settings)) != NULL, OT_NOOP);

            if (message->Append(aData, aDataLength) != OT_ERROR_NONE)     {
                message->Free();
                message = NULL;
            }

            ncp->HandleDatagramFromStack((otMessage*)message);

        exit:
            //buffer_free(buf);
            /* Received Packet gets dropped due to mismatch priority or non availability
             * of message buffers
             */
             // tr_debug("\n Received Packet is dropped due to non-availability of buffers");
            return;
        }


        otError NcpBase::SendQueuedDatagramMessages(void) {
            otError    error = OT_ERROR_NONE;
            otMessage* message;

            while ((message = otMessageQueueGetHead(&mMessageQueue)) != NULL)     {
                // Since an `otMessage` instance can be in one queue at a time,
                // it is first dequeued from `mMessageQueue` before attempting
                // to include it in a spinel frame by calling `SendDatagramMessage()`
                // If forming of the spinel frame fails, the message is enqueued
                // back at the front of `mMessageQueue`.

                otMessageQueueDequeue(&mMessageQueue, message);

                error = SendDatagramMessage(message);

                if (error != OT_ERROR_NONE)         {
                    otMessageQueueEnqueueAtHead(&mMessageQueue, message);
                }

                SuccessOrExit(error);
            }

        exit:
            return error;
        }

        otError NcpBase::SendDatagramMessage(otMessage* aMessage) {
            otError           error = OT_ERROR_NONE;
            uint8_t           header = SPINEL_HEADER_FLAG | SPINEL_HEADER_IID_0;
            bool              isSecure = otMessageIsLinkSecurityEnabled(aMessage);
            spinel_prop_key_t propKey = SPINEL_PROP_STREAM_NET;

            if (!isSecure)     {
                //  tr_debug("\n Error: Attempting to send an un secure IPv6 frame to host");
            }

            SuccessOrExit(error = mEncoder.BeginFrame(header, SPINEL_CMD_PROP_VALUE_IS, propKey));
            SuccessOrExit(error = mEncoder.WriteUint16(otMessageGetLength(aMessage)));
            SuccessOrExit(error = mEncoder.WriteMessage(aMessage));

            // Append any metadata (rssi, lqi, channel, etc) here!

            SuccessOrExit(error = mEncoder.EndFrame());
            mOutboundSecureIpFrameCounter++;

        exit:
            return error;
        }

#ifdef TI_WISUN_FAN_DEBUG
        extern "C" sAddrExt_t mac_eui_filter_list[SIZE_OF_EUI_LIST];

        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_MAC_MAC_FILTER_LIST>(void) {
            otError             error = OT_ERROR_NONE;
            uint16_t index = 0;


            while (index < SIZE_OF_EUI_LIST)     {
                SuccessOrExit(error = mEncoder.WriteEui64(mac_eui_filter_list[index]));
                index++;
            }

        exit:
            return error;
        }


        extern "C" bool macRx_insertAddrIntoList(uint8_t * euiAddress);
        extern "C" bool macRx_removeAddrFromList(uint8_t * euiAddress);
        template <> otError NcpBase::HandlePropertyInsert<SPINEL_PROP_MAC_MAC_FILTER_LIST>(void) {
            otError error = OT_ERROR_NONE;
            bool retVal = false;
            spinel_eui64_t extAddress;

            // read the eui address to be inserted
            SuccessOrExit(error = mDecoder.ReadEui64(extAddress));

            // insert to address list
            retVal = macRx_insertAddrIntoList(&extAddress.bytes[0]);
            if (false == retVal)     {
                error = OT_ERROR_FAILED;
            }

        exit:
            return error;
        }

        template <> otError NcpBase::HandlePropertyRemove<SPINEL_PROP_MAC_MAC_FILTER_LIST>(void) {
            otError error = OT_ERROR_NONE;
            bool retVal = false;
            spinel_eui64_t extAddress;

            // read the eui address to be removed
            SuccessOrExit(error = mDecoder.ReadEui64(extAddress));

            // remove from address list
            retVal = macRx_removeAddrFromList(&extAddress.bytes[0]);
            if (false == retVal)     {
                error = OT_ERROR_NO_ADDRESS;
            }

        exit:
            return error;
        }
#endif //TI_WISUN_FAN_DEBUG

        extern "C"  uint8_t get_current_net_state();
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_NET_STATE>(void) {
            otError error = OT_ERROR_NONE;
            SuccessOrExit(error = mEncoder.WriteUint8(get_current_net_state()));

        exit:
            return error;
        }

        extern "C" if_address_entry_t * get_linkLocal_address();
        extern "C" if_address_entry_t * get_globalUnicast_address();
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_IPV6_ADDRESS_TABLE>(void) {
            otError error = OT_ERROR_NONE;

            if_address_entry_t* entry;

            entry = get_globalUnicast_address();

            if (entry == NULL)     {
                error = OT_ERROR_FAILED;
            }
            SuccessOrExit(error);

            SuccessOrExit(error = mEncoder.OpenStruct());
            SuccessOrExit(error = mEncoder.WriteIp6Address(entry->address));
            SuccessOrExit(error = mEncoder.WriteUint8(entry->prefix_len));
            SuccessOrExit(error = mEncoder.WriteUint32(entry->valid_lifetime));
            SuccessOrExit(error = mEncoder.WriteUint32(entry->preferred_lifetime));
            SuccessOrExit(error = mEncoder.CloseStruct());

            entry = get_linkLocal_address();

            if (entry == NULL)     {
                error = OT_ERROR_FAILED;
            }
            SuccessOrExit(error);

            SuccessOrExit(error = mEncoder.OpenStruct());
            SuccessOrExit(error = mEncoder.WriteIp6Address(entry->address));
            SuccessOrExit(error = mEncoder.WriteUint8(entry->prefix_len));
            SuccessOrExit(error = mEncoder.WriteUint32(entry->valid_lifetime));
            SuccessOrExit(error = mEncoder.WriteUint32(entry->preferred_lifetime));
            SuccessOrExit(error = mEncoder.CloseStruct());

        exit:

            return error;
        }

        extern "C" if_group_list_t * get_multicast_ip_groups();
        template <> otError NcpBase::HandlePropertyGet<SPINEL_PROP_MULTICAST_LIST>(void) {
            otError error = OT_ERROR_NONE;
            if_group_list_t* multicast_list;

            if (get_current_net_state() != 5)     {
                error = OT_ERROR_INVALID_STATE;
            }
            SuccessOrExit(error);

            multicast_list = get_multicast_ip_groups();

            SuccessOrExit(error = mEncoder.OpenStruct());
            ns_list_foreach(if_group_entry_t, entry, multicast_list)     {
                // Only print multicast addresses above realm scope
                if (addr_ipv6_multicast_scope(&entry->group[0]) > IPV6_SCOPE_REALM_LOCAL)         {
                    SuccessOrExit(error = mEncoder.WriteIp6Address(&entry->group[0]));
                }
            }
            SuccessOrExit(error = mEncoder.CloseStruct());

        exit:
            return error;
        }

        extern "C" int8_t add_multicast_addr(const uint8_t * address_ptr);
        template <> otError NcpBase::HandlePropertyInsert<SPINEL_PROP_MULTICAST_LIST>(void) {
            otError error = OT_ERROR_NONE;
            spinel_ipv6addr_t multicast_addr;

            // read the ipv6 address to be inserted
            SuccessOrExit(error = mDecoder.ReadIp6Address(multicast_addr));

            // Return value 0 indicates success
            if (add_multicast_addr(&multicast_addr.bytes[0]) != 0)     {
                error = OT_ERROR_FAILED;
            }

        exit:
            return error;
        }

        extern "C" int8_t remove_multicast_addr(const uint8_t * address_ptr);
        template <> otError NcpBase::HandlePropertyRemove<SPINEL_PROP_MULTICAST_LIST>(void) {
            otError error = OT_ERROR_NONE;
            spinel_ipv6addr_t multicast_addr;

            // read the ipv6 address to be inserted
            SuccessOrExit(error = mDecoder.ReadIp6Address(multicast_addr));

            // Return value 0 indicates success
            if (remove_multicast_addr(&multicast_addr.bytes[0]) != 0)     {
                error = OT_ERROR_FAILED;
            }

        exit:
            return error;
        }


#else // WISUN_NCP_ENABLE
        extern "C" void nanostack_process_routing_table_update_from_stack(uint8_t changed_info, uint8_t * prefix, uint8_t len_prefix, uint8_t * addr_nexthop, uint32_t lifetime) {
            // Stub function
            return;
        }

        extern "C" void nanostack_process_stream_net_from_stack(buffer_t * buf) {
            // Stub function
            return;
        }


#endif //WISUN_NCP_ENABLE



#if __cplusplus >= 201103L
        constexpr bool NcpBase::AreHandlerEntriesSorted(const HandlerEntry* aHandlerEntries, size_t aSize) {
            return aSize < 2 ? true
                : ((aHandlerEntries[aSize - 1].mKey > aHandlerEntries[aSize - 2].mKey) &&
                    AreHandlerEntriesSorted(aHandlerEntries, aSize - 1));
        }

#define OT_NCP_CONST constexpr
#else
#define OT_NCP_CONST const
#endif

        NcpBase::PropertyHandler NcpBase::FindGetPropertyHandler(spinel_prop_key_t aKey) {
#define OT_NCP_GET_HANDLER_ENTRY(aPropertyName) {aPropertyName, &NcpBase::HandlePropertyGet<aPropertyName>}

            OT_NCP_CONST static HandlerEntry sHandlerEntries[] = {
                /* core properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_LAST_STATUS),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PROTOCOL_VERSION),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_NCP_VERSION),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_INTERFACE_TYPE),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_HWADDR),
                /* PHY properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_CCA_THRESHOLD),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_TX_POWER),
                /* MAC properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_MAC_15_4_PANID),
                /* NET properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_NET_IF_UP),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_NET_STACK_UP),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_NET_ROLE),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_NET_NETWORK_NAME),
                /* Tech specific: PHY properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_REGION),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_MODE_ID),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_UNICAST_CHANNEL_LIST),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_BROADCAST_CHANNEL_LIST),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_ASYNC_CHANNEL_LIST),
                /* Tech specific: MAC properties */
                /* Tech specific: NET properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_NET_STATE),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_DODAG_ROUTE),
                //        OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PARENT_LIST),
                //        OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_ROUTING_COST),
                //        OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_LAST_DAO_RCVD_INFO), //mvtodo: revisit
                /* IPv6 properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_IPV6_ADDRESS_TABLE),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_MULTICAST_LIST),
                /* Stream Properties */
                /* Tech specific: PHY Extended properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_CH_SPACING),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_PHY_CHO_CENTER_FREQ),
                /* Tech specific: MAC Extended properties */
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_MAC_UC_DWELL_INTERVAL),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_MAC_BC_DWELL_INTERVAL),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_MAC_BC_INTERVAL),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_MAC_UC_CHANNEL_FUNCTION),
                OT_NCP_GET_HANDLER_ENTRY(SPINEL_MAC_BC_CHANNEL_FUNCTION),
               #ifdef TI_WISUN_FAN_DEBUG
                       OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_MAC_MAC_FILTER_LIST),
                       OT_NCP_GET_HANDLER_ENTRY(SPINEL_PROP_MAC_FILTER_MODE),
               #endif
                       /* Tech specific: NET Extended properties */
            };


//            if(aKey == 95){
//                return OT_NCP_GET_HANDLER_ENTRY();
//                return sHandlerEntries[3].mHandler;
//            }
#undef OT_NCP_GET_HANDLER_ENTRY
            return FindPropertyHandler(sHandlerEntries, OT_ARRAY_LENGTH(sHandlerEntries), aKey);
        }

        NcpBase::PropertyHandler NcpBase::FindSetPropertyHandler(spinel_prop_key_t aKey) {
#define OT_NCP_SET_HANDLER_ENTRY(aPropertyName) {aPropertyName, &NcpBase::HandlePropertySet<aPropertyName>}

            OT_NCP_CONST static HandlerEntry sHandlerEntries[] = {
        #if 0
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_POWER_STATE),
        #if OPENTHREAD_CONFIG_NCP_ENABLE_MCU_POWER_STATE_CONTROL
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_MCU_POWER_STATE),
        #endif
        #if OPENTHREAD_RADIO || OPENTHREAD_CONFIG_LINK_RAW_ENABLE
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_PHY_ENABLED),
        #endif
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_PHY_CHAN),
        #if OPENTHREAD_MTD || OPENTHREAD_FTD
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_PHY_CHAN_SUPPORTED),
        #endif
        #endif
                /* core properties */
                /* PHY properties */
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_PHY_CCA_THRESHOLD),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_PHY_TX_POWER),
                /* MAC properties */
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_MAC_15_4_PANID),
                /* NET properties */
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_NET_IF_UP),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_NET_STACK_UP),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_NET_NETWORK_NAME),
                /* Tech specific: PHY properties */
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_PHY_UNICAST_CHANNEL_LIST),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_PHY_BROADCAST_CHANNEL_LIST),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_PHY_ASYNC_CHANNEL_LIST),
                /* Tech specific: MAC properties */
                /* Tech specific: NET properties */
                /* IPv6 properties */
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_STREAM_NET),
                /* Tech specific: PHY Extended properties */
                /* Tech specific: MAC Extended properties */
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_MAC_UC_DWELL_INTERVAL),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_MAC_BC_DWELL_INTERVAL),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_MAC_BC_INTERVAL),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_MAC_UC_CHANNEL_FUNCTION),
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_MAC_BC_CHANNEL_FUNCTION),
        #ifdef TI_WISUN_FAN_DEBUG
                OT_NCP_SET_HANDLER_ENTRY(SPINEL_PROP_MAC_FILTER_MODE),
        #endif
                /* Tech specific: NET Extended properties */
            };

#undef OT_NCP_SET_HANDLER_ENTRY


            return FindPropertyHandler(sHandlerEntries, OT_ARRAY_LENGTH(sHandlerEntries), aKey);
        }

        NcpBase::PropertyHandler NcpBase::FindInsertPropertyHandler(spinel_prop_key_t aKey) {
#define OT_NCP_INSERT_HANDLER_ENTRY(aPropertyName) {aPropertyName, &NcpBase::HandlePropertyInsert<aPropertyName>}

            OT_NCP_CONST static HandlerEntry sHandlerEntries[] = {
        #ifdef TI_WISUN_FAN_DEBUG
                OT_NCP_INSERT_HANDLER_ENTRY(SPINEL_PROP_MULTICAST_LIST),
                OT_NCP_INSERT_HANDLER_ENTRY(SPINEL_PROP_MAC_MAC_FILTER_LIST),
        #endif
            };

#undef OT_NCP_INSERT_HANDLER_ENTRY


            return FindPropertyHandler(sHandlerEntries, OT_ARRAY_LENGTH(sHandlerEntries), aKey);
        }

        NcpBase::PropertyHandler NcpBase::FindRemovePropertyHandler(spinel_prop_key_t aKey) {
#define OT_NCP_REMOVE_HANDLER_ENTRY(aPropertyName) {aPropertyName, &NcpBase::HandlePropertyRemove<aPropertyName>}

            OT_NCP_CONST static HandlerEntry sHandlerEntries[] = {
        #ifdef TI_WISUN_FAN_DEBUG
                OT_NCP_REMOVE_HANDLER_ENTRY(SPINEL_PROP_MULTICAST_LIST),
                OT_NCP_REMOVE_HANDLER_ENTRY(SPINEL_PROP_MAC_MAC_FILTER_LIST),
        #endif
            };

#undef OT_NCP_REMOVE_HANDLER_ENTRY


            return FindPropertyHandler(sHandlerEntries, OT_ARRAY_LENGTH(sHandlerEntries), aKey);
        }

        NcpBase::PropertyHandler NcpBase::FindPropertyHandler(const HandlerEntry* aHandlerEntries,
            size_t              aSize,
            spinel_prop_key_t   aKey) {
            size_t l = 0;

            OT_ASSERT(aSize > 0);

            for (size_t r = aSize - 1; l < r;) {
                size_t m = (l + r) / 2;

                if (aHandlerEntries[m].mKey < aKey) {
                    l = m + 1;
                }
                else {
                    r = m;
                }
            }

            //return aHandlerEntries[l].mKey == aKey ? aHandlerEntries[l].mHandler : NULL;
            //temporary hack - till other properties are implemented for wisun. Return the last handler entry if property not found
            return aHandlerEntries[l].mKey == aKey ? aHandlerEntries[l].mHandler : aHandlerEntries[(aSize - 1)].mHandler;
        }
    } // name space Ncp
} // name space ot

