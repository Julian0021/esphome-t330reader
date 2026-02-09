# t330reader ESPHome Component

`t330reader` is an external ESPHome sensor platform for Landis+Gyr Ultraheat T330 meters via the optical interface.

## What it does
- Handles meter handshake and data readout.
- Decodes the binary meter payload.
- Publishes clean sensor and text entities.

## Requirements
- Meter optical interface (EN 62056-21).
- UART wiring for RX and TX.

## Add the component
```yaml
external_components:
  - source: github://Julian0021/esphome-t330reader@master
    components: [t330reader]
```

## UART setup
The component expects:
- `2400 8E1` for handshake.
- Internal switch to `9600 8E1` for data phase.

## Exposed service
The component registers one ESPHome API service:
- `start_read_meter`

Behavior:
- Triggers an immediate full meter read cycle (same logic as the periodic `update_interval` read).
- No arguments.
- Useful for manual on-demand reads from Home Assistant or automations.

Optional button example:
```yaml
button:
  - platform: template
    name: "T330 Auslesung starten"
    on_press:
      - lambda: |-
          id(meter_reader).update();
```

## Minimal example
```yaml
esphome:
  name: esp-reader-t330

esp8266:
  board: nodemcuv2

external_components:
  - source: github://Julian0021/esphome-t330reader@master
    components: [t330reader]

logger:
  baud_rate: 0

api: {}
ota:
  - platform: esphome

uart:
  - id: uart_meter
    rx_pin: GPIO3
    tx_pin: GPIO1
    baud_rate: 2400
    data_bits: 8
    parity: EVEN
    stop_bits: 1

sensor:
  - platform: t330reader
    uart_id: uart_meter
    uart_out_id: uart_meter
    update_interval: 60min

    energy_kwh:
      name: "Energie"
    volume_m3:
      name: "Volumen"
    power_kw:
      name: "Leistung"
    flow_rate_m3h:
      name: "Durchfluss"
    flow_temperature_c:
      name: "Vorlauftemperatur"
    return_temperature_c:
      name: "Rücklauftemperatur"
    temperature_difference_k:
      name: "Temperaturdifferenz"
    on_time_h:
      name: "Einschaltzeit"
    operating_time_h:
      name: "Betriebszeit"
    activity_duration_s:
      name: "Aktivitätsdauer"
    averaging_duration_s:
      name: "Mittelungsdauer"
    access_number:
      name: "Zugriffsnummer"
    status:
      name: "Status"
    error_status:
      name: "Fehlerstatus"
    max_power_kw:
      name: "Maximale Leistung"
    max_flow_rate_m3h:
      name: "Maximaler Durchfluss"
    max_flow_temperature_c:
      name: "Maximale Vorlauftemperatur"
    max_return_temperature_c:
      name: "Maximale Rücklauftemperatur"
    previous_energy_kwh:
      name: "Energie Vorperiode"
    previous_volume_m3:
      name: "Volumen Vorperiode"
    previous_on_time_h:
      name: "Einschaltzeit Vorperiode"
    previous_operating_time_h:
      name: "Betriebszeit Vorperiode"

    version_string:
      name: "Version"
    mbus_address:
      name: "M-Bus Adresse"
    fabrication_number:
      name: "Fabrikationsnummer"
    meter_datetime:
      name: "Zählerzeit"
    max_power_datetime:
      name: "Zeitpunkt Maximale Leistung"
    max_flow_rate_datetime:
      name: "Zeitpunkt Maximaler Durchfluss"
    max_flow_temperature_datetime:
      name: "Zeitpunkt Maximale Vorlauftemperatur"
    max_return_temperature_datetime:
      name: "Zeitpunkt Maximale Rücklauftemperatur"
```

## Notes
- `name:` values are optional and can be customized.
- Units, classes, and state classes are defined by the component.
- `error_status` codes:
  - `0`: Read successful
  - `1`: Failed to set input UART to 2400 baud
  - `2`: Handshake failed (after internal retries)
  - `3`: Failed to switch input UART to 9600 baud
  - `4`: No data packets received
  - `5`: Data packets received but none decoded successfully
