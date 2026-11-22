# ESPHome CC1101 Westinghouse Fan

An ESPHome external component for controlling and listening to up to two **Westinghouse Lighting 7316140 ceiling fans** over their 433.92 MHz, 29-bit OOK remote protocol. It exposes native ESPHome fan, button, switch, binary sensor, and diagnostic text sensor entities for Home Assistant.

**Component repository:** [jacekplonka/esphome-cc1101-westinghouse-fan](https://github.com/jacekplonka/esphome-cc1101-westinghouse-fan).

The RF implementation and code generation live in `components/cc1101_fan_controller/`. The YAML file is an example configuration for the two known fan addresses. Change names, addresses, and GPIOs in its `substitutions` and `cc1101_fan_controller` block.

## Features

- Native Home Assistant fan entities with six speed levels.
- Continuous reception of the original remote; the saved command cursor advances when a remote command is heard.
- Two configurable fan addresses, each with independent command cursors and inferred fan/light state.
- Light toggle, color temperature, direction, dimmer, brighter, and seven-second minimum/maximum brightness buttons per fan.
- Maximum/minimum speed actions send five distinct commands in sequence.
- Address discovery switch for finding another remote's 20-bit address.
- Configurable SCK, MISO, MOSI, CS, GDO0, and GDO2 GPIOs.
- ESPHome API, encrypted OTA, Wi-Fi fallback access point, and web server in the example YAML.

Fan state is inferred from commands sent by this controller or received from the original remote. The fan does not provide an RF state response, so Home Assistant cannot confirm that the fan physically accepted a command.

## Hardware

The example defaults target a common ESP32 DevKit using the Arduino framework:

| ESP32 GPIO | CC1101 pin | Use |
| --- | --- | --- |
| GPIO18 | SCK | SPI clock |
| GPIO19 | MISO | SPI data to ESP32 |
| GPIO23 | MOSI | SPI data from ESP32 |
| GPIO5 | CSN | SPI chip select |
| GPIO25 | GDO0 | Transmit data |
| GPIO26 | GDO2 | Receive data |
| 3.3 V | VCC | Power |
| GND | GND | Ground |

Use 3.3 V for the CC1101. Select GPIOs supported by your particular ESP32 board and avoid pins reserved by its flash, PSRAM, USB, or onboard peripherals. Match the six pin substitutions in `fan_controller.yaml` to your wiring.

## Install with Home Assistant ESPHome Device Builder

### Install from GitHub

The example YAML loads the component from `jacekplonka/esphome-cc1101-westinghouse-fan` using ESPHome's [external components feature](https://esphome.io/components/external_components/). Once the repository is public on GitHub:

1. In Home Assistant, open ESPHome Device Builder and create a new ESP32 device or open your existing fan-controller YAML.
2. Copy the contents of `fan_controller.yaml` into that device configuration.
3. Add the secrets shown below to `/config/esphome/secrets.yaml`.
4. Edit the names, addresses, and GPIOs near the top of the YAML.
5. Choose **INSTALL**. For a new board, connect USB and use the browser's serial installer. After the first flash, Device Builder can update it over the network.
6. Accept the discovered ESPHome device in Home Assistant if prompted.

The external component is downloaded by ESPHome during configuration validation/build, so no C++ files need to be copied into the Home Assistant configuration directory.

### Use a local checkout before GitHub publication

Until the repository is available publicly, copy `components/` into `/config/esphome/components/`, copy `fan_controller.yaml` into `/config/esphome/`, and temporarily use this source block:

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [cc1101_fan_controller]
```

Restore the GitHub source block from the example after the repository is public. For a tagged release, replace `@main` with the tag, for example `@v1.0.0`.

## Secrets

Create or edit `secrets.yaml` in the ESPHome configuration directory. Do not commit this file:

```yaml
wifi_ssid: "YOUR_WIFI_NAME"
wifi_password: "YOUR_WIFI_PASSWORD"
fan_controller__encryption_key: "YOUR_UNIQUE_32_BYTE_BASE64_KEY"
fan_controller__fallback_ap_password: "A_UNIQUE_PASSWORD"
```

Use a unique API encryption key per device. For a new device, generate one with `openssl rand -base64 32`; if Home Assistant already provisioned a key for this node, keep using that existing value. The OTA encryption block reuses the API key, following ESPHome's [OTA encryption configuration](https://esphome.io/components/ota/esphome/). The fallback access point is only available when the configured Wi-Fi cannot connect.

## Configure names, addresses, and pins

Edit the substitutions at the start of `fan_controller.yaml`:

```yaml
substitutions:
  fan_1_name: "Office Fan"
  fan_2_name: "Bedroom Fan"
  fan_1_address: "0x94120"
  fan_2_address: "0x64120"
  cc1101_sck_pin: "18"
  cc1101_miso_pin: "19"
  cc1101_mosi_pin: "23"
  cc1101_cs_pin: "5"
  cc1101_gdo0_pin: "25"
  cc1101_gdo2_pin: "26"
```

Addresses are 20-bit values. The example starts with `0x94120` and `0x64120`; use **Fan Address Discovery** and the **Discovered Fan Address** diagnostic entity to identify another address. The current controller supports two configured fans per CC1101.

The `cc1101_fan_controller:` section binds the configured Home Assistant entities to the radio controller. The `fan:`, `button:`, `binary_sensor:`, `switch:`, and `text_sensor:` sections define the entities visible to Home Assistant. Rename the fan entities by changing `fan_1_name` and `fan_2_name`; their button and light-state names follow those substitutions.

## Home Assistant entities

Each fan has:

- One six-speed fan entity, including power control.
- A light-state binary sensor.
- Buttons for light toggle, light color temperature, direction, dimmer, brighter, minimum brightness, and maximum brightness.

The controller also exposes the address-discovery switch and three diagnostic text sensors for the last remote command, last command code, and discovered address. Fan and light state is inferred from RF commands and is not confirmed by a response from the fan.

## Important limits

- The example uses 433.92 MHz ASK/OOK and requires the Arduino framework because the selected CC1101 library is Arduino-based.
- The receiver is continuously active except during transmissions; the controller cannot receive a remote frame while it is sending.
- The fan protocol provides no response frame for state confirmation.
- Yes, the `CC1101-ESP-Arduino` library is fetched as an ESPHome build dependency. The component registers it with `cg.add_library()` in [`components/cc1101_fan_controller/__init__.py`](components/cc1101_fan_controller/__init__.py); ESPHome then adds it to the generated build. That is why it is not listed in the YAML. The configured dependency is [Jacek Plonka's fork](https://github.com/jacekplonka/CC1101-ESP-Arduino), based on the [upstream library by David W. (`wischbgr`)](https://github.com/wischbgr/CC1101-ESP-Arduino); it is not bundled in this repository.

## Credits

- **Shai Dvash** authored the original ESPHome fan-controller migration and initial controller implementation in [esp32-cc1101-rf-caprep](https://github.com/sha1cybr/esp32-cc1101-rf-caprep). This component descends from that work.
- **David W. (`wischbgr`)** authored and maintains the upstream [CC1101-ESP-Arduino library](https://github.com/wischbgr/CC1101-ESP-Arduino). This component uses [Jacek Plonka's fork](https://github.com/jacekplonka/CC1101-ESP-Arduino) as its build dependency; see the upstream repository for license and notices.

## Contributing

Issues and pull requests should include the ESPHome version, board model, CC1101 wiring, fan address, and relevant logs. Do not include Wi-Fi passwords, API keys, or other secrets.
