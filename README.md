# LC3

A `no_std` implementation of the Bluetooth Low Complexity Communication Codec
(LC3) in Rust, following the LC3 specification v1.0 with errata 16703 and
23224, and passing the full LC3 conformance test suite.

- No `std`, no allocation and no `unsafe` code: the whole state of a codec
  lives in its `Encoder` (16.4 KiB) or `Decoder` (23.0 KiB) value.
- All the configurations of the specification: 8, 16, 24, 32, 44.1 and
  48 kHz, 7.5 and 10 ms frames, 20 to 400 bytes per frame, changing from
  frame to frame if needed.
- 16, 24 and 32 bits integer samples and floating point samples, in mono or
  interleaved buffers.
- Packet loss concealment of lost frames and of frames with bit errors.

## Usage

```rust
use lc3::{Config, Decoder, Encoder, FrameDuration, SampleRate};

let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
let mut encoder = Encoder::new(config);
let mut decoder = Decoder::new(config);

let pcm = [0i16; 480]; // config.frame_samples()
let mut frame = [0u8; 120]; // config.frame_bytes(96_000)
encoder.encode(&pcm, &mut frame)?;

let mut output = [0i16; 480];
decoder.decode(Some(&frame), &mut output)?; // `None` for a lost frame
```

An encoder or a decoder codes a single channel: multi-channel audio is coded
with one per channel, `encode_interleaved` and `decode_interleaved` reading
and writing interleaved buffers. The decoded signal is delayed by
`config.delay_samples()` samples.

## Command line

The `cli` feature builds the `lc3` command, which encodes WAVE files to LC3
bitstream files and decodes them, with the command line and file format of
the reference executable of the conformance software:

```bash
cargo build --release --features cli
```

```bash
target/release/lc3 -E input.wav encoded.bin 96000
```

```bash
target/release/lc3 -D encoded.bin decoded.wav
```

`lc3 -h` lists the options: frame duration, bitrate switching, frame loss
patterns and output resolution.

## Conformance

All the test cases of the LC3 Test Suite pass with the conformance script of
the LC3 conformance interoperability test software V1.0.8, with gstPEAQ for
the perceptual measurements (see [conformance/](conformance/README.md)):

| Tests | Criterion | Worst result |
| --- | --- | --- |
| Decoder, 10 and 7.5 ms | RMS difference ≤ -89.06 dB, max. difference ≤ 0.00148 | -90.32 dB, 0.00046 |
| Encoder, 10 ms | ODG difference ≤ 0.07 | 0.020 |
| Encoder, 7.5 ms | ODG difference ≤ 0.07 | 0.047 |
| Encoder and decoder, 10 ms | ODG difference ≤ 0.07 | 0.031 |
| Encoder and decoder, 7.5 ms | ODG difference ≤ 0.07 | 0.054 |
| Encoder low pass, 10 and 7.5 ms | attenuation < 70 dB | 65.95 dB, 69.63 dB |

`cargo test --all-features` checks the codec against the intermediate data
and the encoded frames of appendix C of the specification, and the public API
over all the configurations.

## Development

```bash
cargo test --all-features
```

```bash
cargo clippy --all-features --all-targets
```

```bash
cargo build --lib --target thumbv7em-none-eabihf
```

## License

MIT, see [LICENSE](LICENSE). The documents of `docs/` and the conformance
software of `third_party/` belong to their respective owners.
