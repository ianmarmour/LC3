# Conformance testing

`run.sh` runs the official LC3 conformance script (`conformanceCheck.py`,
from the conformance software in `third_party/`) on the `lc3` command, with
one of the configurations of this directory:

| Configuration | Test cases |
| --- | --- |
| `lc3_10ms.cfg`, `lc3_75ms.cfg` | Encoder, decoder and encoder-decoder tests of the LC3 Test Suite, for 10 ms and 7.5 ms frames |
| `lc3_decoder_10ms.cfg`, `lc3_decoder_75ms.cfg` | Decoder tests only, which need no PEAQ tool |

```bash
conformance/run.sh lc3_decoder_10ms.cfg
```

The working directory defaults to `target/conformance`. The first run
downloads the EBU SQAM items. The script writes an HTML report per test to
the working directory and prints whether the tests passed.

## Requirements

- python3 with numpy, to run the conformance script,
- wine able to run the 32 bits reference executable `LC3.exe` (`wine` and
  `wine32:i386` on Debian and Ubuntu),
- sox with FLAC support, gcc and curl,
- for the encoder and encoder-decoder tests,
  [gstPEAQ](https://github.com/HSU-ANT/gstpeaq) 0.6.1: `lc3_10ms.cfg` and
  `lc3_75ms.cfg` expect `peaq` and `libgstpeaq.so` in the working directory.
  The Test Suite allows gstPEAQ with an ODG threshold of 0.07 instead of 0.06,
  set by `DEFAULTS[mode + '_odg_threshold']` in `conformanceCheck.py`.

`run.sh` passes `-system_sox` to use the SoX of the system rather than the
Windows binary that the script downloads otherwise, and works around the
`gcc -lm rms.c -o rms` command of the script, which fails to link with
toolchains defaulting to `--as-needed`.
