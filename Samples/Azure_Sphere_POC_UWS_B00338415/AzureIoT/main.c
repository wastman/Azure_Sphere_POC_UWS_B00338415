/*
Proof of concept for a heating control system (20th of June 2019)

This is POC makes use of following examples and modifies them for this POC:
1) MT3620_Grove_Shield_Library
https://github.com/Seeed-Studio/MT3620_Grove_Shield/tree/master/MT3620_Grove_Shield_Library
2) Microsoft Sample for the MT3620 Developement Board (Sample: Azure IoT)
https://github.com/Azure/azure-sphere-samples/tree/master/Samples/WifiSetupAndDeviceControlViaBle
3) Microsoft Sample for the MT3620 Developement Board (Sample: UART)
https://github.com/Azure/azure-sphere-samples/tree/master/Samples/WifiSetupAndDeviceControlViaBle

Student ID: b00338415
University: University West of Scotland
E-Mail: b00338415@studentmail.uws.ac.uk
*/

/* Microsoft Azure Sphere Sample License:
Copyright (c) Microsoft Corporation. All rights reserved.
Licensed under the MIT License. */

// You will need to provide four pieces of information to use this application, all of which are set
// in the app_manifest.json.
// 1. The Scope Id for your IoT Central application (set in 'CmdArgs')
// 2. The Tenant Id obtained from 'azsphere tenant show-selected' (set in 'DeviceAuthentication')
// 3. The Azure DPS Global endpoint address 'global.azure-devices-provisioning.net' (set in 'AllowedConnections')
// 4. The IoT Hub Endpoint address for your IoT Central application (set in 'AllowedConnections')

#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/networking.h>
#include <applibs/gpio.h>
#include <applibs/storage.h>

// By default, this sample is targeted at the MT3620 Reference Development Board (RDB).
#include <hw\sample_hardware.h>
#include "epoll_timerfd_utilities.h"

// Azure IoT SDK
#include <iothub_client_core_common.h>
#include <iothub_device_client_ll.h>
#include <iothub_client_options.h>
#include <iothubtransportmqtt.h>
#include <iothub.h>
#include <azure_sphere_provisioning.h>

// Library for Groove Shield Development Board 
#include "mt3620_rdb.h"
#include "Grove.h"
#include "Sensors/GroveTempHumiSHT31.h"
#include "Sensors/GroveRotaryAngleSensor.h"
#include "Sensors/GroveOledDisplay96x96.h"

static volatile sig_atomic_t terminationRequired = false;

#include "parson.h" // used to parse Device Twin messages.

// Azure IoT Hub/Central defines.
#define SCOPEID_LENGTH 20
static char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application, set in
                                     // app_manifest.json, CmdArgs

static IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle = NULL;
static const int keepalivePeriodSeconds = 20;
static bool iothubAuthenticated = false;
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context);
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload, size_t payloadSize, void *userContextCallback);
static void TwinReportBoolState(const char *propertyName, bool propertyValue);
static void ReportStatusCallback(int result, void *context);
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason);
static const char *getAzureSphereProvisioningResultString(AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult);
static void SendTelemetry(const unsigned char *key, const unsigned char *value);
static void SetupAzureClient(void);

// Function to generate simulated Temperature data/telemetry
static void SendData(void);

// Initialization/Cleanup
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value
static int uartFd = -1;

// File descriptors - initialized to invalid value
// Buttons
static int sendButtonAGpioFd = -1;
static int sendButtonBGpioFd = -1;
static int sendAutomaticStatusButtonGpioFd = -1;

// Heater Relay Switch
static int deviceTwinHeatSwitchGpioFd = -1;
static bool statusHeatOn = false;

// Automatic Manuell Blue LED Button (Button Pin)
static int deviceAutomaticButtonLedGpioFd = -1;

// Automatic Manuell Blue LED Button (LED Pin)
static int deviceHeatSwitchStatusLedGpioFd = -1;

// Should Temp Variable - preset to 19 Degress
static int shouldTemp = 19;

// Timer / polling
static int buttonPollTimerFd = -1;
static int azureTimerFd = -1;
static int epollFd = -1;

// Azure IoT poll periods
static const int AzureIoTDefaultPollPeriodSeconds = 5;
static const int AzureIoTMinReconnectPeriodSeconds = 60;
static const int AzureIoTMaxReconnectPeriodSeconds = 10 * 60;

static int azureIoTPollPeriodSeconds = -1;

// Button state variables
static GPIO_Value_Type sendButtonAState = GPIO_Value_High;
static GPIO_Value_Type sendButtonBState = GPIO_Value_High;
static GPIO_Value_Type sendAutomaticButtonState = GPIO_Value_High;

static void ButtonPollTimerEventHandler(EventData *eventData);
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState);
static void SendButtonAHandler(void);
static void SendButtonBHandler(void);
static void SendAutomaticButtonHandler(void);
static bool autoClimaIsOn = false; 
static void AzureTimerEventHandler(EventData *eventData);

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    terminationRequired = true;
}

/// <summary>
///     Main entry point for Azure Sphere POC UWS B00338415 Application
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("Azure Sphere POC UWS B00338415 Application starting.\n");

    if (argc == 2) {
        Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
        strncpy(scopeId, argv[1], SCOPEID_LENGTH);
    } else {
        Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
        return -1;
    }

    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

	setGrayLevel(10); 
	setTextXY(1, 0);  
	putString("Azure Sphere POC"); 
	setTextXY(2, 0);  
	putString("MaibornWolff"); 
	setTextXY(4, 0);
	putString("Temperature:");
	setTextXY(6, 0);
	putString("Humidity:");
	setTextXY(11, 0);
	putString("Should-Temp");
	setTextXY(15, 0);
	putString("Firmware V1.1");

    // Main loop
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return 0;
}

/// <summary>
/// Button timer event:  Check the status of all buttons (A, B, automatic)
/// </summary>
static void ButtonPollTimerEventHandler(EventData *eventData)
{
    if (ConsumeTimerFdEvent(buttonPollTimerFd) != 0) {
        terminationRequired = true;
        return;
    }
    SendButtonAHandler();
	SendButtonBHandler();
    SendAutomaticButtonHandler();
}

/// <summary>
/// Azure timer event:  Check connection status and send telemetry
/// </summary>
static void AzureTimerEventHandler(EventData *eventData)
{
    if (ConsumeTimerFdEvent(azureTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

    bool isNetworkReady = false;
    if (Networking_IsNetworkingReady(&isNetworkReady) != -1) {
        if (isNetworkReady && !iothubAuthenticated) {
            SetupAzureClient();
        }
    } else {
        Log_Debug("Failed to get Network state\n");
    }

    if (iothubAuthenticated) {
        SendData();
        IoTHubDeviceClient_LL_DoWork(iothubClientHandle);
    }
}

// event handler data structures. Only the event handler field needs to be populated.
static EventData buttonPollEventData = {.eventHandler = &ButtonPollTimerEventHandler};
static EventData azureEventData = {.eventHandler = &AzureTimerEventHandler};

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

	// Open UART for MT3620 Grove Shield
	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 9600;
	uartConfig.flowControl = UART_FlowControl_None;
	uartFd = UART_Open(UART_Grove_Shield, &uartConfig);
	if (uartFd < 0) {
		Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	// OLED Display Init (Grove - OLED Display 1.12" V2)
	GroveOledDisplay_Init(UART_Grove_Shield, SH1107G);
	clearDisplay();
	setNormalDisplay();
	setVerticalMode();

    // Open button A GPIO as input
    Log_Debug("Opening BUTTON_A (down temp) as input\n");
    sendButtonAGpioFd = GPIO_OpenAsInput(BUTTON_A);
    if (sendButtonAGpioFd < 0) {
        Log_Debug("ERROR: Could not open button A: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

	// Open button B GPIO as input
	Log_Debug("Opening BUTTON_B (down temp) as input\n");
	sendButtonBGpioFd = GPIO_OpenAsInput(BUTTON_B);
	if (sendButtonBGpioFd < 0) {
		Log_Debug("ERROR: Could not open button B: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	// Open button connected to GPIO0 MT3620 Grove Shield (Grove - Blue LED Button)
    Log_Debug("Opening AUTOMATIC_BUTTON as input\n");
    sendAutomaticStatusButtonGpioFd = GPIO_OpenAsInput(AUTOMATIC_BUTTON);
    if (sendAutomaticStatusButtonGpioFd < 0) {
        Log_Debug("ERROR: Could not open button B: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

	// Open LED connected to GPIO0 MT3620 Grove Shield (Grove - Blue LED Button)
	Log_Debug("Opening AUTOMATIC_BUTTON_LED as output\n");
	deviceAutomaticButtonLedGpioFd =
		GPIO_OpenAsOutput(AUTOMATIC_BUTTON_LED, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (deviceAutomaticButtonLedGpioFd < 0) {
		Log_Debug("ERROR: Could not open Automatic Button Led: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

    // Open GPIO for Heater Switch connected to GPIO4 MT3620 Grove Shield (Grove - Relay)
    Log_Debug("Opening HEAT_Switch as output\n");
    deviceTwinHeatSwitchGpioFd =
        GPIO_OpenAsOutput(HEAT_SWITCH, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (deviceTwinHeatSwitchGpioFd < 0) {
        Log_Debug("ERROR: Could not open HeatSwitch: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

	// Heat Switch Status LED (LED1_Red on MT3620 Developer Board)
	Log_Debug("Opening HEAT_LED as output\n");
	deviceHeatSwitchStatusLedGpioFd =
		GPIO_OpenAsOutput(HEAT_SWITCH_STATUS_LED, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (deviceHeatSwitchStatusLedGpioFd < 0) {
		Log_Debug("ERROR: Could not open Heat Switch Status Led: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

    // Set up a timer to poll for button events.
    struct timespec buttonPressCheckPeriod = {0, 1000 * 1000};
    buttonPollTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &buttonPressCheckPeriod, &buttonPollEventData, EPOLLIN);
    if (buttonPollTimerFd < 0) {
        return -1;
    }

    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
    azureTimerFd =
        CreateTimerFdAndAddToEpoll(epollFd, &azureTelemetryPeriod, &azureEventData, EPOLLIN);
    if (buttonPollTimerFd < 0) {
        return -1;
    }

    return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    Log_Debug("Closing file descriptors\n");

    // Leave the LEDs OFF
	if (deviceAutomaticButtonLedGpioFd >= 0) {
		GPIO_SetValue(deviceAutomaticButtonLedGpioFd, GPIO_Value_Low);
	}
	if (deviceHeatSwitchStatusLedGpioFd >= 0) {
		GPIO_SetValue(deviceHeatSwitchStatusLedGpioFd, GPIO_Value_Low);
	}
	// Leave Heat Switch OFF
	if (deviceTwinHeatSwitchGpioFd >= 0) {
		GPIO_SetValue(deviceTwinHeatSwitchGpioFd, GPIO_Value_Low);
	}
    CloseFdAndPrintError(buttonPollTimerFd, "ButtonTimer");
    CloseFdAndPrintError(azureTimerFd, "AzureTimer");
    CloseFdAndPrintError(sendButtonAGpioFd, "SendButtonA");
	CloseFdAndPrintError(sendButtonBGpioFd, "SendButtonB");
    CloseFdAndPrintError(sendAutomaticStatusButtonGpioFd, "AutomaticStatusButton");
	CloseFdAndPrintError(deviceTwinHeatSwitchGpioFd, "HeatSwitch");
	CloseFdAndPrintError(deviceAutomaticButtonLedGpioFd, "AutomaticButtonLed");
	CloseFdAndPrintError(deviceHeatSwitchStatusLedGpioFd, "HeatSwitchStatusLed");
    CloseFdAndPrintError(epollFd, "Epoll");
}

/// <summary>
///     Sets the IoT Hub authentication state for the app
///     The SAS Token expires which will set the authentication state
/// </summary>
static void HubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,void *userContextCallback)
{
    iothubAuthenticated = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    Log_Debug("IoT Hub Authenticated: %s\n", GetReasonString(reason));
}

/// <summary>
///     Sets up the Azure IoT Hub connection (creates the iothubClientHandle)
///     When the SAS Token for a device expires the connection needs to be recreated
///     which is why this is not simply a one time call.
/// </summary>
static void SetupAzureClient(void)
{
    if (iothubClientHandle != NULL)
        IoTHubDeviceClient_LL_Destroy(iothubClientHandle);

    AZURE_SPHERE_PROV_RETURN_VALUE provResult =
        IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning(scopeId, 10000,
                                                                          &iothubClientHandle);
    Log_Debug("IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning returned '%s'.\n",
              getAzureSphereProvisioningResultString(provResult));

    if (provResult.result != AZURE_SPHERE_PROV_RESULT_OK) {

        // If we fail to connect, reduce the polling frequency, starting at
        // AzureIoTMinReconnectPeriodSeconds and with a backoff up to
        // AzureIoTMaxReconnectPeriodSeconds
        if (azureIoTPollPeriodSeconds == AzureIoTDefaultPollPeriodSeconds) {
            azureIoTPollPeriodSeconds = AzureIoTMinReconnectPeriodSeconds;
        } else {
            azureIoTPollPeriodSeconds *= 2;
            if (azureIoTPollPeriodSeconds > AzureIoTMaxReconnectPeriodSeconds) {
                azureIoTPollPeriodSeconds = AzureIoTMaxReconnectPeriodSeconds;
            }
        }

        struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
        SetTimerFdToPeriod(azureTimerFd, &azureTelemetryPeriod);

        Log_Debug("ERROR: failure to create IoTHub Handle - will retry in %i seconds.\n",
                  azureIoTPollPeriodSeconds);
        return;
    }

    // Successfully connected, so make sure the polling frequency is back to the default
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
    SetTimerFdToPeriod(azureTimerFd, &azureTelemetryPeriod);

    iothubAuthenticated = true;

    if (IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_KEEP_ALIVE,
                                        &keepalivePeriodSeconds) != IOTHUB_CLIENT_OK) {
        Log_Debug("ERROR: failure setting option \"%s\"\n", OPTION_KEEP_ALIVE);
        return;
    }

    IoTHubDeviceClient_LL_SetDeviceTwinCallback(iothubClientHandle, TwinCallback, NULL);
    IoTHubDeviceClient_LL_SetConnectionStatusCallback(iothubClientHandle,
                                                      HubConnectionStatusCallback, NULL);
}

/// <summary>
///     Callback invoked when a Device Twin update is received from IoT Hub.
///     Updates local state for 'showEvents' (bool).
/// </summary>
/// <param name="payload">contains the Device Twin JSON document (desired and reported)</param>
/// <param name="payloadSize">size of the Device Twin JSON document</param>
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload,
                         size_t payloadSize, void *userContextCallback)
{
    size_t nullTerminatedJsonSize = payloadSize + 1;
    char *nullTerminatedJsonString = (char *)malloc(nullTerminatedJsonSize);
    if (nullTerminatedJsonString == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for twin update payload.\n");
        abort();
    }

    // Copy the provided buffer to a null terminated buffer.
    memcpy(nullTerminatedJsonString, payload, payloadSize);
    // Add the null terminator at the end.
    nullTerminatedJsonString[nullTerminatedJsonSize - 1] = 0;

    JSON_Value *rootProperties = NULL;
    rootProperties = json_parse_string(nullTerminatedJsonString);
    if (rootProperties == NULL) {
        Log_Debug("WARNING: Cannot parse the string as JSON content.\n");
        goto cleanup;
    }

    JSON_Object *rootObject = json_value_get_object(rootProperties);
    JSON_Object *desiredProperties = json_object_dotget_object(rootObject, "desired");
    if (desiredProperties == NULL) {
        desiredProperties = rootObject;
    }

    // Handle the Device Twin Desired Properties here.
    JSON_Object *HeatState = json_object_dotget_object(desiredProperties, "SwitchHEAT");
    if (HeatState != NULL) {
        statusHeatOn = (bool)json_object_get_boolean(HeatState, "value");
			if (statusHeatOn) {
				GPIO_SetValue(deviceTwinHeatSwitchGpioFd, GPIO_Value_High);
				GPIO_SetValue(deviceHeatSwitchStatusLedGpioFd, GPIO_Value_Low);
				SendTelemetry("HeaterRelay", "On");
				setTextXY(9, 0);
				putString("Heat On ");
			}
			else {
				GPIO_SetValue(deviceTwinHeatSwitchGpioFd, GPIO_Value_Low);
				GPIO_SetValue(deviceHeatSwitchStatusLedGpioFd, GPIO_Value_High);
				SendTelemetry("HeaterRelay", "Off");
				setTextXY(9, 0);
				putString("Heat Off");
			};
		TwinReportBoolState("SwitchHEAT", statusHeatOn);
    }

cleanup:
    // Release the allocated memory.
    json_value_free(rootProperties);
    free(nullTerminatedJsonString);
}

/// <summary>
///     Converts the IoT Hub connection status reason to a string.
/// </summary>
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason)
{
    static char *reasonString = "unknown reason";
    switch (reason) {
    case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:
        reasonString = "IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN";
        break;
    case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED";
        break;
    case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:
        reasonString = "IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL";
        break;
    case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED";
        break;
    case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_NO_NETWORK";
        break;
    case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:
        reasonString = "IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR";
        break;
    case IOTHUB_CLIENT_CONNECTION_OK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_OK";
        break;
    }
    return reasonString;
}

/// <summary>
///     Converts AZURE_SPHERE_PROV_RETURN_VALUE to a string.
/// </summary>
static const char *getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult)
{
    switch (provisioningResult.result) {
    case AZURE_SPHERE_PROV_RESULT_OK:
        return "AZURE_SPHERE_PROV_RESULT_OK";
    case AZURE_SPHERE_PROV_RESULT_INVALID_PARAM:
        return "AZURE_SPHERE_PROV_RESULT_INVALID_PARAM";
    case AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR";
    case AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR";
    default:
        return "UNKNOWN_RETURN_VALUE";
    }
}

/// <summary>
///     Sends data to IoT Hub
/// </summary>
/// <param name="key">The telemetry item to update</param>
/// <param name="value">new telemetry value</param>
static void SendTelemetry(const unsigned char *key, const unsigned char *value)
{
    static char eventBuffer[100] = {0};
    static const char *EventMsgTemplate = "{ \"%s\": \"%s\" }";
    int len = snprintf(eventBuffer, sizeof(eventBuffer), EventMsgTemplate, key, value);
    if (len < 0)
        return;

    Log_Debug("Sending IoT Hub Message: %s\n", eventBuffer);

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(eventBuffer);

    if (messageHandle == 0) {
        Log_Debug("WARNING: unable to create a new IoTHubMessage\n");
        return;
    }

    if (IoTHubDeviceClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendMessageCallback,
                                             /*&callback_param*/ 0) != IOTHUB_CLIENT_OK) {
        Log_Debug("WARNING: failed to hand over the message to IoTHubClient\n");
    } else {
        Log_Debug("INFO: IoTHubClient accepted the message for delivery\n");
    }

    IoTHubMessage_Destroy(messageHandle);
}

/// <summary>
///     Callback confirming message delivered to IoT Hub.
/// </summary>
/// <param name="result">Message delivery status</param>
/// <param name="context">User specified context</param>
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context)
{
    Log_Debug("INFO: Message received by IoT Hub. Result is: %d\n", result);
}

/// <summary>
///     Creates and enqueues a report containing the name and value pair of a Device Twin reported
///     property. The report is not sent immediately, but it is sent on the next invocation of
///     IoTHubDeviceClient_LL_DoWork().
/// </summary>
/// <param name="propertyName">the IoT Hub Device Twin property name</param>
/// <param name="propertyValue">the IoT Hub Device Twin property value</param>
static void TwinReportBoolState(const char *propertyName, bool propertyValue)
{
    if (iothubClientHandle == NULL) {
        Log_Debug("ERROR: client not initialized\n");
    } else {
        static char reportedPropertiesString[30] = {0};
        int len = snprintf(reportedPropertiesString, 30, "{\"%s\":%s}", propertyName,
                           (propertyValue == true ? "true" : "false"));
        if (len < 0)
            return;

        if (IoTHubDeviceClient_LL_SendReportedState(
                iothubClientHandle, (unsigned char *)reportedPropertiesString,
                strlen(reportedPropertiesString), ReportStatusCallback, 0) != IOTHUB_CLIENT_OK) {
            Log_Debug("ERROR: failed to set reported state for '%s'.\n", propertyName);
        } else {
            Log_Debug("INFO: Reported state for '%s' to value '%s'.\n", propertyName,
                      (propertyValue == true ? "true" : "false"));
        }
    }
}

/// <summary>
///     Callback invoked when the Device Twin reported properties are accepted by IoT Hub.
/// </summary>
static void ReportStatusCallback(int result, void *context)
{
    Log_Debug("INFO: Device Twin reported properties update result: HTTP status code %d\n", result);
}

/// <summary>
///     Sends data (temperature&humidity)
/// </summary>
void SendData(void)
{
	// Read Sensor via UART=>i2C
	void* sht31 = GroveTempHumiSHT31_Open(UART_Grove_Shield);
	GroveTempHumiSHT31_Read(sht31);

	// Should Temperature
	char shouldTempBuffer[20];
	int len = snprintf(shouldTempBuffer, 20, "%d", shouldTemp);
	Log_Debug("Should Temperature: %s\n", shouldTempBuffer);
	setTextXY(12, 0);
	putString(shouldTempBuffer);
	SendTelemetry("ShouldTemperature", shouldTempBuffer);
	
	// Temperature
	float temp = GroveTempHumiSHT31_GetTemperature(sht31);
	char tempBuffer[20];
	len = snprintf(tempBuffer, 20, "%3.2f", temp);
	Log_Debug("Temperature: %s\n", tempBuffer);
	setTextXY(5, 0);
	putString(tempBuffer);
	SendTelemetry("Temperature", tempBuffer);

	// Humidity
	float humi = GroveTempHumiSHT31_GetHumidity(sht31);
	char humiBuffer[20];
	len = snprintf(humiBuffer, 20, "%3.2f", humi);
	Log_Debug("Humidity: %s\n", humiBuffer);
	setTextXY(7, 0);
	putString(humiBuffer);
	SendTelemetry("Humidity", humiBuffer);
}

/// <summary>
///     Check whether a given button has just been pressed.
/// </summary>
/// <param name="fd">The button file descriptor</param>
/// <param name="oldState">Old state of the button (pressed or released)</param>
/// <returns>true if pressed, false otherwise</returns>
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState)
{
    bool isButtonPressed = false;
    GPIO_Value_Type newState;
    int result = GPIO_GetValue(fd, &newState);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
        terminationRequired = true;
    } else {
        // Button is pressed if it is low and different than last known state.
        isButtonPressed = (newState != *oldState) && (newState == GPIO_Value_Low);
        *oldState = newState;
    }

    return isButtonPressed;
}

/// <summary>
/// Pressing button A will: lower the should temperature 
/// </summary>
static void SendButtonAHandler(void)
{
    if (IsButtonPressed(sendButtonAGpioFd, &sendButtonAState)) {
		shouldTemp = shouldTemp - 1;
    }
}

/// <summary>
/// Pressing button B will:
/// Pressing button A will: raise the should temperature 
/// </summary>
static void SendButtonBHandler(void)
{
	if (IsButtonPressed(sendButtonBGpioFd, &sendButtonBState)) {
		shouldTemp = shouldTemp + 1;
	}
}

/// <summary>
/// Pressing button automatic will:
///     Send status event to Azure IoT Central
/// </summary>
static void SendAutomaticButtonHandler(void)
{
    if (IsButtonPressed(sendAutomaticStatusButtonGpioFd, &sendAutomaticButtonState)) {
		autoClimaIsOn = !autoClimaIsOn;
		if (autoClimaIsOn) {
			SendTelemetry("AutomaticClimatisation","On");
			GPIO_SetValue(deviceAutomaticButtonLedGpioFd, GPIO_Value_High);
			setTextXY(8, 0);
			putString("Automatic ");
		} else {
			SendTelemetry("AutomaticClimatisation", "Off");
			GPIO_SetValue(deviceAutomaticButtonLedGpioFd, GPIO_Value_Low);
			setTextXY(8, 0);
			putString("Manuell   ");
		};
    }

}
