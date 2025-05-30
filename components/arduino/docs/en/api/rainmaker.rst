#############
ESP Rainmaker
#############

About
-----

This library allows to work with ESP RainMaker.

ESP RainMaker is an end-to-end solution offered by Espressif to enable remote control and monitoring for ESP32-S2 and ESP32 based products without any configuration required in the Cloud. The primary components of this solution are:

- Claiming Service (to get the Cloud connectivity credentials)
- RainMaker library (i.e. this library, to develop the firmware)
- RainMaker Cloud (backend, offering remote connectivity)
- RainMaker Phone App/CLI (Client utilities for remote access)

The key features of ESP RainMaker are:

1. Ability to define own devices and parameters, of any type, in the firmware.
2. Zero configuration required on the Cloud.
3. Phone apps that dynamically render the UI as per the device information.

Additional information about ESP RainMaker can be found `here <https://rainmaker.espressif.com/>`__.


ESP RainMaker Agent API
-----------------------

RMaker.initNode
***************

This initializes the ESP RainMaker agent, Wi-Fi and creates the node.

You can also set the configuration of the node using the following API

``RMaker.setTimeSync(bool val)``

**NOTE**: If you want to set the configuration for the node then these configuration API must be called before `RMaker.initNode()`.

.. code-block:: arduino

    Node initNode(const char *name, const char *type);

* ``name`` Name of the node
* ``type`` Type of the node

This function will return object of Node.

RMaker.start
************

It starts the ESP RainMaker agent.

**NOTE**:

1. ESP RainMaker agent should be initialized before this call.
2. Once ESP RainMaker agent starts, compulsorily call ``WiFi.beginProvision()`` API.

.. code-block:: arduino

    esp_err_t start();

This function will return `ESP_OK` on success or  `Error` in case of failure.

RMaker.stop
***********

It stops the ESP RainMaker agent which was started using `RMaker.start()`.

.. code-block:: arduino

    esp_err_t stop()

This function will return

1. `ESP_OK` : On success
2. Error in case of failure.

RMaker.deinitNode
*****************

It deinitializes the ESP RainMaker agent and the node created using `RMaker.initNode()`.

.. code-block:: arduino

    esp_err_t deinitNode(Node node)

* ``node`` : Node object created using `RMaker.initNode()`

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

RMaker.enableOTA
****************

It enables OTA as per the ESP RainMaker Specification. For more details refer ESP RainMaker documentation. check `here <https://rainmaker.espressif.com/docs/ota.html>`__.

.. code-block:: arduino

    esp_err_t enableOTA(ota_type_t type);

* ``type`` : The OTA workflow type.
    - OTA_USING_PARAMS
    - OTA_USING_TOPICS

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

RMaker.enableSchedule
*********************

This API enables the scheduling service for the node. For more information, check `here <https://rainmaker.espressif.com/docs/scheduling.html>`__.

.. code-block:: arduino

    esp_err_t enableSchedule();

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

RMaker.enableScenes
*******************

This API enables the Scenes service for the node. It should be called after `RMaker.initNode()` and before `RMaker.start()`.
For more information, check `here <https://rainmaker.espressif.com/docs/scenes.html>`__.

.. code-block:: arduino

    esp_err_t enableScenes()

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

RMaker.enableSystemService
**************************

This API enables the System service for the node. It should be called after `RMaker.initNode()` and before `RMaker.start()`.
For more information, check `here <https://rainmaker.espressif.com/docs/sys-service.html>`__.

.. code-block:: arduino

    esp_err_t enableSystemService(uint16_t flags, int8_t reboot_seconds, int8_t reset_seconds, int8_t reset_reboot_seconds)

* ``flags`` : Logical OR of system service flags (SYSTEM_SERV_FLAG_REBOOT, SYSTEM_SERV_FLAG_FACTORY_RESET, SYSTEM_SERV_FLAG_WIFI_RESET) as required or SYSTEM_SERV_FLAGS_ALL.
* ``reboot_seconds`` Time in seconds after which the device should reboot. Recommended value: 2
* ``reset_seconds`` Time in seconds after which the device should reset(Wi-Fi or Factory). Recommended value: 2
* ``reset_reboot_seconds`` Time in seconds after which the device should reboot after it has been reset. Zero as a value would mean there won't be any reboot after the reset. Recommended value: 2

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

RMaker.setTimeZone
******************

This API set's the timezone as a user friendly location string. Check
`here <https://rainmaker.espressif.com/docs/time-service.html>`__ for a list of valid values.

**NOTE** : default value is "Asia/Shanghai".

This API comes into picture only when working with scheduling.

.. code-block:: arduino

    esp_err_t setTimeZone(const char *tz);

* ``tz`` : Valid values as specified in documentation.

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

ESP RainMaker Node API
-----------------------

`Node` class expose API's for node.

**NOTE** : my_node is the object of Node class.

my_node.getNodeID
*****************

It returns the unique node_id assigned to the node. This node_id is usually the MAC address of the board.

.. code-block:: arduino

    char * getNodeID()

* ``tz`` : Valid values as specified in documentation.

This function will return

1. `char *` : Pointer to a NULL terminated node_id string.

my_node.getNodeInfo
*******************

It returns pointer to the node_info_t as configured during node initialization.

.. code-block:: arduino

    node_info_t * getNodeInfo();

This function will return

1. `node_info_t` : Pointer to the structure node_info_t on success.
2. `NULL` : On failure.

**ESP RainMaker node info**

It has following data member

1. char * name
2. char * type
3. char * fw_version
4. char * model

my_node.addNodeAttr
*******************

It adds a new attribute as the metadata to the node.

**NOTE** : Only string values are allowed.

.. code-block:: arduino

    esp_err_t addNodeAttr(const char *attr_name, const char *val);

* ``attr_name`` : Name of the attribute
* ``val`` : Value of the attribute

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

my_node.addDevice
*****************

It adds a device to the node.

**NOTE** :

- This is the mandatory API to register device to node.
- Single Node can have multiple devices.
- Device name should be unique for each device.

.. code-block:: arduino

    esp_err_t addDevice(Device device);

* ``device`` : Device object

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

my_node.removeDevice
********************

It removes a device from the node.

.. code-block:: arduino

    esp_err_t removeDevice(Device device);

* ``device`` : Device object

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

ESP RainMaker Device API
-----------------------------

`Device` class expose API's for virtual devices on the node.
Parameterized constructor is defined which creates the virtual device on the node. Using Device class object you can create your own device.

**NOTE** : my_device is the object of Device class

.. code-block:: arduino

    Device my_device(const char *dev_name, const char *dev_type, void *priv_data);

* ``dev_name`` : Unique device name
* ``dev_type`` : Optional device type. It can be kept NULL.
    * Standard Device Types
        * ESP_RMAKER_DEVICE_SWITCH
        * ESP_RMAKER_DEVICE_LIGHTBULB
        * ESP_RMAKER_DEVICE_FAN
        * ESP_RMAKER_DEVICE_TEMP_SENSOR
* ``priv_data`` : Private data associated with the device. This will be passed to the callbacks.

**NOTE** : This created device should be added to the node using ``my_node.addDevice(my_device);``

- Sample example

.. code-block:: arduino

    Device my_device("Switch");
    Device my_device("Switch1", NULL, NULL);

- Here, dev_name is compulsory, rest are optional.
- Node can have multiple device, each device should have unique device name.

**Standard Devices**

- Classes are defined for the standard devices.
- Creating object of these class creates the standard device with default parameters to it.
- Class for standard devices
    * Switch
    * LightBulb
    * TemperatureSensor
    * Fan

.. code-block:: arduino

    Switch my_switch(const char *dev_name, void *priv_data, bool power);

* ``dev_name`` : Unique device name by default it is "switch" for switch device.
* ``priv_data`` : Private data associated with the device. This will be passed to the callbacks.
* ``power`` : It is the value that can be set for primary parameter.

Sample example for standard device.

.. code-block:: arduino

    Switch switch1;
    Switch switch2("switch2", NULL, true);

- `"switch2"` : Name for standard device.
- `NULL` : Private data for the device, which will be used in callback.
- `true` : Default value for the primary param, in case of switch it is power.

**NOTE**: No parameter are compulsory for standard devices. However if you are creating two objects of same standard class then in that case you will have to set the device name, if not then both device will have same name which is set by default, hence device will not get create. *Device name should be unique for each device.*

my_device.getDeviceName
***********************

It returns the name of the Device.

.. code-block:: arduino

    const char * getDeviceName();

* ``device`` : Device object

This function will return

- `char *`: Returns Device name.

**NOTE**: Each device on the node should have unique device name.

my_device.addDeviceAttr
***********************

It adds attribute to the device. Device attributes are reported only once after a boot-up as part of the node configuration. Eg. Serial Number

.. code-block:: arduino

    esp_err_t addDeviceAttr(const char *attr_name, const char *val);

* ``attr_name`` : Name of the attribute
* ``val`` : Value of the attribute

This function will return

1. `ESP_OK` : On success
2. Error in case  of failure

my_device.deleteDevice
**********************

It deletes the device created using parameterized constructor.

This device should be first removed from the node using `my_node.removeDevice(my_device)`.

.. code-block:: arduino

    esp_err_t deleteDevice();

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

my_device.addXParam
*******************

It adds standard parameter to the device.

**NOTE**: X is the default name by which parameter is referred, you can specify your own name to each parameter.

- Eg. `my_device.addPowerParam(true)` here power parameter is referred with name Power.
- Eg. `my_device.addHueParam(12)` here hue parameter is referred with name Hue.

You can specify your own name to each parameter

- Eg. `my_device.addNameParam("NickName")` here name parameter is referred with name NickName.
- Eg. `my_device.addPowerParam(true, "FanPower")` here power parameter is referred with name FanPower.

**Standard Parameters**

* These are the standard parameters.
    * Name : ESP_RMAKER_DEF_NAME_PARAM
    * Power : ESP_RMAKER_DEF_POWER_NAME
    * Brightness : ESP_RMAKER_DEF_BRIGHTNESS_NAME
    * Hue : ESP_RMAKER_DEF_HUE_NAME
    * Saturation : ESP_RMAKER_DEF_SATURATION_NAME
    * Intensity : ESP_RMAKER_DEF_INTENSITY_NAME
    * CCT : ESP_RMAKER_DEF_CCT_NAME
    * Direction : ESP_RMAKER_DEF_DIRECTION_NAME
    * Speed : ESP_RMAKER_DEF_SPEED_NAME
    * Temperature : ESP_RMAKER_DEF_TEMPERATURE_NAME

.. code-block:: arduino

    esp_err_t addNameParam(const char *param_name = ESP_RMAKER_DEF_NAME_PARAM);
    esp_err_t addPowerParam(bool val, const char *param_name = ESP_RMAKER_DEF_POWER_NAME);
    esp_err_t addBrightnessParam(int val, const char *param_name = ESP_RMAKER_DEF_BRIGHTNESS_NAME);
    esp_err_t addHueParam(int val, const char *param_name = ESP_RMAKER_DEF_HUE_NAME);
    esp_err_t addSaturationParam(int val, const char *param_name = ESP_RMAKER_DEF_SATURATION_NAME);
    esp_err_t addIntensityParam(int val, const char *param_name = ESP_RMAKER_DEF_INTENSITY_NAME);
    esp_err_t addCCTParam(int val, const char *param_name = ESP_RMAKER_DEF_CCT_NAME);
    esp_err_t addDirectionParam(int val, const char *param_name = ESP_RMAKER_DEF_DIRECTION_NAME);
    esp_err_t addSpeedParam(int val, const char *param_name = ESP_RMAKER_DEF_SPEED_NAME);
    esp_err_t addTempratureParam(float val, const char *param_name = ESP_RMAKER_DEF_TEMPERATURE_NAME);

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

**NOTE** : Care should be taken while accessing name of parameter. Above mentioned are the two ways using which default name of parameters can be accessed. Either LHS or RHS.

my_device.assignPrimaryParam
****************************

It assigns a parameter (already added using addXParam() or addParam()) as a primary parameter, which can be used by clients (phone apps specifically) to give prominence to it.

.. code-block:: arduino

    esp_err_t assignPrimaryParam(param_handle_t *param);

* ``param`` : Handle of the parameter. It is obtained using `my_device.getParamByName()`.

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

my_device.getParamByName
************************

.. code-block:: arduino

    param_handle_t * getParamByName(const char *param_name);

* ``param_name`` : It is the name of the parameter which was added using addXparam() or addParam().

This function will return object of the parameter.

my_device.addParam
******************

It allows user to add custom parameter to the device created using `Param` class.

.. code-block:: arduino

    esp_err_t addParam(Param parameter);

* ``parameter`` : Object of Param

This function will return

1.`ESP_OK` : On success
2. Error in case of failure

**NOTE**: Param class exposes API's to create the custom parameter.

my_device.updateAndReportParam
******************************

It updates the parameter assosicated with particular device on ESP RainMaker cloud.

.. code-block:: arduino

    esp_err_t updateAndReportParam(const char *param_name, value);

* ``param_name`` : Name of the parameter
* ``value`` : Value to be updated. It can be int, bool, char * , float.

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

my_device.addCb
***************

It registers read and write callback for the device which will be invoked as per requests received from the cloud (or other paths as may be added in future).

.. code-block:: arduino

    void addCb(deviceWriteCb write_cb, deviceReadCb read_cb);

* ``write_cb`` : Function with signature
        func_name(Device \*device, Param \*param, const param_val_t val, void \*priv_data, write_ctx_t \*ctx);
* ``read_cb`` : Function with signature
        func_name(Device \*device, Param \*param, void \*priv_data, read_ctx_t \*ctx);

**Parameters**

**param_val_t val**

Value can be accessed as below

1. `bool` : val.val.b
2. `integer` : val.val.i
3. `float` : val.val.f
4. `char *` : val.val.s

ESP RainMaker Param API
-----------------------

`Param` class expose API's for creating custom parameters for the devices and report and update values associated with parameter to the ESP RainMaker cloud. Parameterized constructor is defined which creates custom parameter.

**NOTE** : `my_param` is the object of Param class.

.. code-block:: arduino

    Param my_param(const char *param_name, const char *param_type, param_val_t val, uint8_t properties);

* ``param_name`` : Name of the parameter
* ``param_type`` : Type of the parameter. It is optional can be kept NULL.
* ``val`` : Define the default value for the parameter. It should be defined using `value(int ival)` , `value(bool bval)` , `value(float fval)` , `value(char *sval)`.
* ``properties`` : Properties of the parameter, which will be a logical OR of flags.
    * Flags
        * PROP_FLAG_WRITE
        * PROP_FLAG_READ
        * PROP_FLAG_TIME_SERIES
        * PROP_FLAG_PERSIST

Sample example :

.. code-block:: arduino

    Param my_param(const char *param_name, const char *param_type, param_val_t val, uint8_t properties);
    Param my_param("bright", NULL, value(30), PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);

**NOTE** : Parameter created using Param class should be added to the device using `my_device.addParam(my_param);`

my_param.addUIType
******************

Add a UI type to the parameter. This will be used by the Phone apps (or other clients) to render appropriate UI for the given parameter. Please refer the RainMaker documentation
`here <https://rainmaker.espressif.com/docs/standard-types.html#ui-elements>`__ for supported UI Types.

.. code-block:: arduino

    esp_err_t addUIType(const char *ui_type);

* ``ui_type`` : String describing the UI Type.
    * Standard UI Types
        * ESP_RMAKER_UI_TOGGLE
        * ESP_RMAKER_UI_SLIDER
        * ESP_RMAKER_UI_DROPDOWN
        * ESP_RMAKER_UI_TEXT

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

my_param.addBounds
******************

Add bounds for an integer/float parameter. This can be used to add bounds (min/max values) for a given integer/float parameter. Eg. brightness will have bounds as 0 and 100 if it is a percentage.

.. code-block:: arduino

    esp_err_t addBounds(param_val_t min, param_val_t max, param_val_t step);

* ``min`` : Minimum value
* ``max`` : Maximum value
* ``step`` : step Minimum stepping

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

`Sample example : my_param.addBounds(value(0), value(100), value(5));`

my_param.updateAndReport
************************

It updates the parameter and report it to ESP RainMaker cloud. This is called in callback.

.. code-block:: arduino

    esp_err_t updateAndReport(param_val_t val);

* ``val`` : New value of the parameter

This function will return

1. `ESP_OK` : On success
2. Error in case of failure

**NOTE**:

- This API should always be called inside device write callback, if you aimed at updating n reporting parameter values, changed via RainMaker Client (Phone App), to the ESP RainMaker cloud.
- If not called then parameter values will not be updated to the ESP RainMaker cloud.

printQR
*******

This API displays QR code, which is used in provisioning.

.. code-block:: arduino

    printQR(const char *serv_name, const char *pop, const char *transport);

* ``name`` : Service name used in provisioning API.
* ``pop`` : Proof of possession used in provisioning API.
* ``transport`` :
    1. `softap` : In case of provisioning using SOFTAP.
    2. `ble` : In case of provisioning using BLE.

RMakerFactoryReset
******************

Reset the device to factory defaults.

.. code-block:: arduino

    RMakerFactoryReset(int seconds);

* ``seconds`` : Time in seconds after which the chip should reboot after doing a factory reset.

RMakerWiFiReset
***************

Reset Wi-Fi credentials.

.. code-block:: arduino

    RMakerWiFiReset(int seconds);

* ``seconds`` : Time in seconds after which the chip should reboot after doing a Wi-Fi reset.
