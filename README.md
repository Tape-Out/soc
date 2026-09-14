# soc

SoC assembly framework: the shared skeleton every soc-* reference builds on.

![maturity](https://img.shields.io/badge/maturity-planned-lightgrey) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Planned, not started. It is meant to become the parameterised SoC generator; work starts when the skeleton shared by the reference SoCs can be pulled out. The Verilog from the picorv32 era that sits here is kept as history only; the implementation will be written from scratch in Bluespec.

## License

Mulan PSL v2 for our own sources. See `third_party/` for anything else.
