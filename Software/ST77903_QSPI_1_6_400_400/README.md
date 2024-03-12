| Supported Targets | ESP32-C6 | ESP32-S3 |
| ----------------- | -------- | -------- |

# QSPI LCD Example

This example shows how to use ST77903 display driver. These components are using API provided by `esp_lcd` component. This example will display LVGL demos. For more information about porting the LVGL library, you can also refer to [another lvgl porting example](../i80_controller/README.md).

**Important Notes**:

1. This example is only applicable to ST77903 QSPI LCD and may not be suitable for other QSPI interface LCDs, such as SPD2010. For more information about ST77903, please refer to the [docs](./docs).
2. This example is still under development and relies on some new features that have not yet been merged into ESP-IDF. To successfully compile it, you will need to apply the patch, see [here](#idf-requirements).

## IDF Requirements

* ESP-IDF: **master** branch, use `git checkout --recurse-submodules c133949da6` switch to the commit **c133949da6**
* Patch: go to the root folder of ESP-IDF, use `git apply <path_of_patch>/spi_support_segment_mode_c133949da6.patch` to apply the [patch](patch/spi_support_segment_mode_c133949da6.patch)

## How to use the example

### Hardware Required

* An ESP development board
* An ST77903 LCD panel, with QSPI interface
* An USB cable for power supply and programming

### Hardware Connection

The connection between ESP Board and the LCD is as follows:

```
       ESP Board                       ST77903 Panel (QSPI)
┌──────────────────────┐              ┌────────────────────┐
│             GND      ├─────────────►│ GND                │
│                      │              │                    │
│             3V3      ├─────────────►│ VCC                │
│                      │              │                    │
│             GND      ├─────────────►│ RS                 │
│                      │              │                    │
│             CS       ├─────────────►│ CS                 │
│                      │              │                    │
│             SCK      ├─────────────►│ CLK                │
│                      │              │                    │
│             D3       ├─────────────►│ IO3                │
│                      │              │                    │
│             D2       ├─────────────►│ IO2                │
│                      │              │                    │
│             D1       ├─────────────►│ IO1                │
│                      │              │                    │
│             D0       ├─────────────►│ IO0                │
│                      │              │                    │
│             RST      ├─────────────►│ RSTN               │
│                      │              │                    │
│             BK_LIGHT ├─────────────►│ BLK                │
└──────────────────────┘              └────────────────────┘
```

The LCD parameters and GPIO number used by this example can be changed in [lv_port.h](main/lv_port.h).
Especially, please pay attention to the **vendor specific initialization**, it can be different between manufacturers and should consult the LCD supplier for initialization sequence code. You can change the code `vendor_specific_init` in [esp_lcd_st77903_qspi.c](components/esp_lcd_st77903_qspi/esp_lcd_st77903_qspi.c).

### Build and Flash

Run `idf.py set-target esp32s3` or  `idf.py set-target esp32c6` to choose the target chip.

Run `idf.py -p PORT build flash monitor` to build, flash and monitor the project. A fancy animation will show up on the LCD as expected.

The first time you run `idf.py` for the example will cost extra time as the build system needs to address the component dependencies and downloads the missing components from registry into `managed_components` folder.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

### Example Output

```bash
...
I (534) gpio: GPIO[5]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (543) lcd_panel.st77903_qspi: SPI max transfer size: 639KB
I (550) lcd_panel.st77903_qspi: Init SPI bus[1]
I (555) lcd_panel.st77903_qspi: Add SPI device success
I (586) lcd_panel.st77903_qspi: Frame buffer size: 320000, total: 625KB
I (586) lcd_panel.st77903_qspi: Bounce buffer size: 16000, total: 31KB
I (589) lcd_panel.st77903_qspi: Trans pool size: 20, total: 1KB
I (596) lcd_panel.st77903_qspi: segment_gap_clock_len: 0
I (602) lcd_panel.st77903_qspi: segment_interval(us): 42, refresh_delay(ms): 0
I (610) lcd_panel.st77903_qspi: version: 0.1.0
I (745) lcd_panel.st77903_qspi: Load memory task start
I (868) lcd_panel.st77903_qspi: Refresh task start
I (872) lv_port: Starting LVGL task
I (941) main_task: Returned from app_main()
I (2767) lcd_panel.st77903_qspi: FPS: 52
...
```

## Troubleshooting

For any technical queries, please open an [issue] (https://github.com/espressif/esp-dev-kits/issues) on GitHub. We will get back to you soon.
