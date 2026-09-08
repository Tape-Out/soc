# soc

SoC assembly framework: the shared skeleton every soc-* reference builds on.

![maturity](https://img.shields.io/badge/maturity-planned-lightgrey) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`loom`](https://github.com/Tape-Out/loom). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Planned. What sits in this repository today is the retired picorv32-era Verilog, kept for
provenance. The Bluespec rewrite against the [`spec`](https://github.com/Tape-Out/spec)
contracts has not landed yet, and it will not reuse this source.

`mysoc32.v` is the retired top level: a picorv32 core wired to CLINT, PLIC, UART, PWM,
GPIO, MROM, WDT and Timer. `third_party/` holds the core itself, unmodified, under its
original ISC licence.

The replacement is not another hand-written top level. Assembly becomes declarative:
an `ip.yaml` with an `instances:` section, from which the top level is generated. See
[`spec`](https://github.com/Tape-Out/spec).

## License

Mulan PSL v2 for our own sources. See `third_party/` for anything else.
