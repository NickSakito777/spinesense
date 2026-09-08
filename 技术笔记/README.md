# Technical notes

English translations of the seven historical hardware notes retained in this repository. Original filenames are kept so existing links remain valid. The dates and experiment status belong to the original notes; translation does not establish a new hardware validation.

For the repaired garment and September 2026 acquisition kit, follow the [Tom / Southampton operating guide](../docs/tom-handoff/README.md). In particular, use that kit’s matching firmware, TA0 wiring and acceptance procedure. The notes below preserve earlier hypotheses, failed approaches and design proposals.

| Note | Scope |
|---|---|
| [IMU selection: six axes, nine axes and BNO086](IMU%E9%80%89%E5%9E%8B%20-%206%E8%BD%B4vs9%E8%BD%B4%E4%B8%8EBNO086.md) | Sensor trade-offs and historical selection rationale |
| [STEVAL-MKI248KA commercial IMU kit specifications](STEVAL-MKI248KA%20%E5%95%86%E4%B8%9A%E7%89%88IMU%E8%A7%84%E6%A0%BC.md) | Evaluation kit interfaces, adapter and electrical details |
| [IMU fan-out board design brief](IMU%20Fanout%20Board%20%E8%AE%BE%E8%AE%A1%E9%9C%80%E6%B1%82.md) | Historical passive-board proposal and routing requirements |
| [Pull-ups and the I3C physical layer — 18 May 2026](%E4%B8%8A%E6%8B%89%E7%94%B5%E9%98%BB%E4%B8%8E%20I3C%20%E7%89%A9%E7%90%86%E5%B1%82%E8%B0%83%E7%A0%94%202026-05-18.md) | Pull-up behaviour, electrical constraints and source references |
| [stm32duino I3C feasibility — 22 May 2026](stm32duino%20I3C%20%E5%8F%AF%E8%A1%8C%E6%80%A7%E8%B0%83%E7%A0%94%202026-05-22.md) | Arduino/HAL integration, code examples and development routes |
| [Connecting two I3C IMUs — 22 May 2026](I3C%20%E4%B8%A4%E9%A2%97%20IMU%20%E8%BF%9E%E6%8E%A5%E8%B0%83%E7%A0%94%202026-05-22.md) | Two-device starting test |
| [Multi-IMU I3C bring-up — 22 May 2026](I3C%20%E5%A4%9A%E9%A2%97%20IMU%20bring-up%20%E5%AE%9E%E6%93%8D%202026-05-22.md) | Progressive three-to-five-device wiring and validation |

References to private work logs are provenance citations only; those logs and participant recordings are not included in this public repository.
