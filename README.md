## Task A — Fake accelerometer (out‑of‑tree module)

### What you build

A minimal driver that implements the **Zephyr Sensor API** for a fake 3‑axis accelerometer producing simulated/dummy axis data on `native_posix`.

### Requirements

- **Module layout (in its own git repo that you will add to west manifest to be fetched with west update):**
  ```
  sensor_xyz/
    ├─ zephyr/module.yml
    ├─ CMakeLists.txt
    ├─ Kconfig
    ├─ dts/bindings/sensor,xyz.yaml
    ├─ include/sensor_xyz.h
    └─ drivers/sensor/sensor_xyz.c
  ```
- `` (minimal):
  ```yaml
  build:
    cmake: .
    kconfig: Kconfig
  ```
- **Kconfig options:**
  - `CONFIG_SENSOR_XYZ` (bool) — enable driver
  - `CONFIG_SENSOR_XYZ_RATE_HZ` (int, default 100, range 1..1600) — ODR
- **Devicetree binding (**``**):**
  - `compatible: "sensor,xyz"`
  - `properties:` `label` (string), `odr-hz` (int, default 100)
- **Driver behavior:**
  - Implements `struct sensor_driver_api` with `.sample_fetch` and `.channel_get`.
  - Supports `SENSOR_CHAN_ACCEL_XYZ`.
  - Generates simulated/dummy samples: e.g., a slow sine/cosine on X/Y, constant Z = 9.80665 m/s², or however you see fit.
  - Timestamps internally with `k_uptime_get()` (or expose timestamp via your app’s buffer).
  - Optional `pm_device` hooks that pause the timer when suspended.
- **Sampling strategy:**
  - A kernel timer/work item produces a new sample at `CONFIG_SENSOR_XYZ_RATE_HZ` into internal state; `.sample_fetch` latches the latest.
- **Units:**
  - Use `struct sensor_value` with values in **m/s²** for channel SENSOR_CHAN_ACCEL_XYZ as per Zephyr Sensor API documentation.