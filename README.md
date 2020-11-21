# DCMotor firmware

> This repo contains an alternative firmware for the DCMotor controller board written in Rust

[`probe-run`]: https://crates.io/crates/probe-run
[`defmt`]: https://github.com/knurling-rs/defmt
[`flip-link`]: https://github.com/knurling-rs/flip-link

## Dependencies

#### 1. `flip-link`:
```console
$ cargo install flip-link
```

#### 2. The **git** version of `probe-run`:

<!-- TODO: update this once defmt is on crates.io? -->
```console
$ cargo install probe-run
```

#### 3. Install toolchain
```console
$ rustup target add thumbv6m-none-eabi
```

#### 4. Useful tools:
```console
$ cargo install cargo-binutils cargo-bloat
```
#### 5. Udev rules

Setup [udev rules](https://probe.rs/guide/2_probes/udev/)

## Usage

#### 1. Clone the code
#### 2. Modify CANOpen ID in `main.rs`
#### 3. Connect the driver board
#### 4. Run

```bash
cargo run --release
```

## Acknowledgements

DCMotor firmware was developed using tools that are part of the [Knurling] project, [Ferrous Systems]' effort at improving tooling used to develop for embedded systems. Various libraries contributed by the Rust and embedded Rust communities were also used.

Development of the firmware was sponsored by the Robotics and AI research group at Faculty of Electrical Engineering and Communications at Brno University of Technology.

## License
MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

[Knurling]: https://github.com/knurling-rs/meta
[Ferrous Systems]: https://ferrous-systems.com/
