//! Command line arguments, compatible with the reference executable of the
//! LC3 conformance software.

use std::fmt;
use std::path::PathBuf;

use lc3::FrameDuration;

/// Help of the command.
pub(crate) const USAGE: &str = "\
Usage:
  lc3 [OPTIONS] INPUT.wav OUTPUT.wav BITRATE     Encode and decode
  lc3 -E [OPTIONS] INPUT.wav OUTPUT.bin BITRATE  Encode
  lc3 -D [OPTIONS] INPUT.bin OUTPUT.wav          Decode

BITRATE is a bitrate in bits per second, or a file of bitrates stored as
little-endian 64 bits integers, one per frame (looped when shorter than the
input).

Options:
  -frame_ms MS   Frame duration, 10 (default) or 7.5
  -dc N          Delay compensation: 0 none, 1 in the decoder (default)
  -epf FILE      Frame loss pattern of the decoder: 16 bits values, non-zero
                 for a lost frame (looped when shorter than the bitstream)
  -bps N         Bits per sample of the decoded output, 16 (default), 24 or 32
  -q             Quiet: only print errors
  -v             Verbose: also print debugging messages
  -h             Print this help
";

/// Bitrate of the encoded frames.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Bitrate {
    /// Fixed bitrate, in bits per second.
    Fixed(u32),
    /// File of the bitrate of each frame.
    Switching(PathBuf),
}

/// Operation of the command.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Mode {
    /// Encodes a WAVE file and decodes the result to a WAVE file.
    EncodeDecode(Bitrate),
    /// Encodes a WAVE file to an LC3 bitstream file.
    Encode(Bitrate),
    /// Decodes an LC3 bitstream file to a WAVE file.
    Decode,
}

/// Compensation of the codec delay in the decoded output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum DelayCompensation {
    /// The decoded output is delayed relative to the input.
    None,
    /// The decoder drops the first samples, aligning its output with the input.
    Decoder,
}

/// Sample format of the decoded output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum OutputFormat {
    /// 16 bits integers.
    Int16,
    /// 24 bits integers.
    Int24,
    /// 32 bits integers.
    Int32,
}

/// Amount of messages printed by the command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Verbosity {
    /// Errors only.
    Quiet,
    /// Errors, warnings and a summary of the operation.
    Normal,
    /// All messages, including debugging messages.
    Verbose,
}

/// Arguments of the command.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Args {
    /// Operation.
    pub(crate) mode: Mode,
    /// Frame duration of the encoder.
    pub(crate) frame_duration: FrameDuration,
    /// Delay compensation of the decoder.
    pub(crate) delay_compensation: DelayCompensation,
    /// Frame loss pattern file of the decoder.
    pub(crate) loss_pattern: Option<PathBuf>,
    /// Sample format of the decoded output.
    pub(crate) output_format: OutputFormat,
    /// Amount of messages.
    pub(crate) verbosity: Verbosity,
    /// Input file.
    pub(crate) input: PathBuf,
    /// Output file.
    pub(crate) output: PathBuf,
}

/// Invalid command line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ArgsError {
    /// The help is requested.
    Help,
    /// An option is not followed by its value.
    MissingValue(&'static str),
    /// The value of an option is not supported.
    InvalidValue(&'static str, String),
    /// An option is unknown.
    UnknownOption(String),
    /// `-E` and `-D` are both given.
    ExclusiveModes,
    /// The number of positional arguments does not match the operation.
    ArgumentCount,
}

impl fmt::Display for ArgsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Help => f.write_str("help requested"),
            Self::MissingValue(option) => write!(f, "missing value of {option}"),
            Self::InvalidValue(option, value) => write!(f, "unsupported value {value} of {option}"),
            Self::UnknownOption(option) => write!(f, "unknown option {option}"),
            Self::ExclusiveModes => f.write_str("-E and -D are exclusive"),
            Self::ArgumentCount => f.write_str("invalid number of arguments"),
        }
    }
}

impl std::error::Error for ArgsError {}

impl Args {
    /// Parses the arguments of the command, without the program name.
    pub(crate) fn parse(args: impl IntoIterator<Item = String>) -> Result<Self, ArgsError> {
        let mut args = args.into_iter();
        let (mut encode, mut decode) = (false, false);
        let mut frame_duration = FrameDuration::Ms10;
        let mut delay_compensation = DelayCompensation::Decoder;
        let mut loss_pattern = None;
        let mut output_format = OutputFormat::Int16;
        let mut verbosity = Verbosity::Normal;
        let mut positional = Vec::new();

        while let Some(arg) = args.next() {
            let mut value = |option| args.next().ok_or(ArgsError::MissingValue(option));
            match arg.as_str() {
                "-E" => encode = true,
                "-D" => decode = true,
                "-q" => verbosity = Verbosity::Quiet,
                "-v" => verbosity = Verbosity::Verbose,
                "-h" | "--help" => return Err(ArgsError::Help),
                "-frame_ms" => {
                    frame_duration = match value("-frame_ms")?.as_str() {
                        "10" | "10.0" => FrameDuration::Ms10,
                        "7.5" | "7.50" => FrameDuration::Ms7_5,
                        other => return Err(ArgsError::InvalidValue("-frame_ms", other.into())),
                    };
                }
                "-dc" => {
                    delay_compensation = match value("-dc")?.as_str() {
                        "0" => DelayCompensation::None,
                        "1" => DelayCompensation::Decoder,
                        other => return Err(ArgsError::InvalidValue("-dc", other.into())),
                    };
                }
                "-epf" => loss_pattern = Some(PathBuf::from(value("-epf")?)),
                "-bps" => {
                    output_format = match value("-bps")?.as_str() {
                        "16" => OutputFormat::Int16,
                        "24" => OutputFormat::Int24,
                        "32" => OutputFormat::Int32,
                        other => return Err(ArgsError::InvalidValue("-bps", other.into())),
                    };
                }
                option if option.len() > 1 && option.starts_with('-') => {
                    return Err(ArgsError::UnknownOption(arg));
                }
                _ => positional.push(arg),
            }
        }

        let mut positional = positional.into_iter();
        let (Some(input), Some(output)) = (positional.next(), positional.next()) else {
            return Err(ArgsError::ArgumentCount);
        };
        let bitrate = positional.next().map(|bitrate| match bitrate.parse() {
            Ok(bps) => Bitrate::Fixed(bps),
            Err(_) => Bitrate::Switching(PathBuf::from(bitrate)),
        });
        if positional.next().is_some() {
            return Err(ArgsError::ArgumentCount);
        }
        let mode = match (encode, decode, bitrate) {
            (true, true, _) => return Err(ArgsError::ExclusiveModes),
            (false, true, None) => Mode::Decode,
            (true, false, Some(bitrate)) => Mode::Encode(bitrate),
            (false, false, Some(bitrate)) => Mode::EncodeDecode(bitrate),
            _ => return Err(ArgsError::ArgumentCount),
        };

        Ok(Self {
            mode,
            frame_duration,
            delay_compensation,
            loss_pattern,
            output_format,
            verbosity,
            input: PathBuf::from(input),
            output: PathBuf::from(output),
        })
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::{Args, ArgsError, Bitrate, DelayCompensation, Mode, OutputFormat, Verbosity};
    use lc3::FrameDuration;

    /// Parses a command line.
    fn parse(line: &str) -> Result<Args, ArgsError> {
        Args::parse(line.split_whitespace().map(String::from))
    }

    #[test]
    fn parses_the_conformance_encoder_command() {
        let args = parse("-E -frame_ms 7.5 in.wav out.bin 32000").unwrap();
        assert_eq!(args.mode, Mode::Encode(Bitrate::Fixed(32_000)));
        assert_eq!(args.frame_duration, FrameDuration::Ms7_5);
        assert_eq!(args.input, PathBuf::from("in.wav"));
        assert_eq!(args.output, PathBuf::from("out.bin"));
    }

    #[test]
    fn parses_the_conformance_decoder_command() {
        let args = parse("-D -q -bps 24 -dc 0 -epf loss.dat in.bin out.wav").unwrap();
        assert_eq!(args.mode, Mode::Decode);
        assert_eq!(args.output_format, OutputFormat::Int24);
        assert_eq!(args.delay_compensation, DelayCompensation::None);
        assert_eq!(args.loss_pattern, Some(PathBuf::from("loss.dat")));
        assert_eq!(args.verbosity, Verbosity::Quiet);
    }

    #[test]
    fn bitrate_switching_files() {
        let args = parse("in.wav out.wav swf.dat").unwrap();
        assert_eq!(
            args.mode,
            Mode::EncodeDecode(Bitrate::Switching("swf.dat".into()))
        );
    }

    #[test]
    fn rejects_invalid_command_lines() {
        assert_eq!(parse("-h").unwrap_err(), ArgsError::Help);
        assert_eq!(parse("-E -D a b 1").unwrap_err(), ArgsError::ExclusiveModes);
        assert_eq!(parse("-D a b 1").unwrap_err(), ArgsError::ArgumentCount);
        assert_eq!(parse("-E a b").unwrap_err(), ArgsError::ArgumentCount);
        assert_eq!(parse("a b c d").unwrap_err(), ArgsError::ArgumentCount);
        assert_eq!(
            parse("a b 1 -frame_ms").unwrap_err(),
            ArgsError::MissingValue("-frame_ms")
        );
        assert_eq!(
            parse("-frame_ms 5 a b 1").unwrap_err(),
            ArgsError::InvalidValue("-frame_ms", "5".into())
        );
        assert_eq!(
            parse("-x a b 1").unwrap_err(),
            ArgsError::UnknownOption("-x".into())
        );
    }
}
