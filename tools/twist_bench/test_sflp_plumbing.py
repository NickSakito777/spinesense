from __future__ import annotations

import numpy as np

import five_imu_fusion as fiv
import twist_bench_v0 as v0


def main() -> int:
    raw = "0 IMU0 rear 0x6A 1 2 999 0.1 0.2 0.3\n"
    sflp = "10 IMU0 rear 0x6A 1 2 999 0.1 0.2 0.3 1 0 0 0\n"

    raw_record = v0.parse_serial_text(raw)[0]
    sflp_record = v0.parse_serial_text(sflp)[0]
    assert raw_record.sflp_quat is None
    assert sflp_record.sflp_quat == (1.0, 0.0, 0.0, 0.0)

    rows = v0.read_dict_rows("t_ms,imu,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,qw,qx,qy,qz\n"
                             "20,IMU1,1,2,999,0.1,0.2,0.3,0.7071,0.7071,0,0\n")
    assert v0.parse_long_table_rows(rows)[0].sflp_quat == (0.7071, 0.7071, 0.0, 0.0)

    result = fiv.run_pipeline(None, fiv.make_args(demo=True, filter="sflp"))
    assert result.summary["algorithm"].endswith("_sflp")
    for state in result.sensors.values():
        assert state.q_filter.shape == (len(result.t_s), 4)
        assert np.allclose(np.linalg.norm(state.q_filter, axis=1), 1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
