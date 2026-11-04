# RPM conversion (`ThrusterRpmConverter`)

Converts a thruster force `F` (N) into rpm and saturates it to `[rpm_conversion.min_rpm, rpm_conversion.max_rpm]`.

| `rpm_conversion.mode` | Used when | Model |
|---|---|---|
| (absent) / static curve | `thrust` allocation | `F = coef[0] rpm^2 + coef[1] rpm + coef[2]`, inverted for rpm, with `coef_fwd` for `F >= 0` and `coef_bwd` for `F < 0` |
| 0 | `thrust_rudder` | propeller model `F = rho (rpm/60)^2 D^4 K_T_BP (1 - u / (rps * prop_pitch))` using the surge velocity through water `u` |
| 1 | `thrust_rudder` | the static polynomial curve above |

Parameters: `coef_fwd[3]`, `coef_bwd[3]`, `max_rpm`, `min_rpm` (always), and for the propeller model `rho` (1025 kg/m^3), `K_T_BP`, `prop_pitch` (m, 0.381 = 15 in), `D` (m, 0.4318). Examples: delfim `coef_fwd = [1.77778e-6, 0, 0]`, limits +-5000 rpm; magicelectric mode 0, limits +-2200 rpm.

Note: the `open_loop` node of `farol2_inner_loop` bypasses this converter and does not apply these limits.
