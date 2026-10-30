//! LTPF decoder: the pitch postfilter (section 3.4.9).

use super::tables::{
    DENOMINATOR_8000, DENOMINATOR_16000, DENOMINATOR_24000, DENOMINATOR_32000, DENOMINATOR_48000,
    NUMERATOR_8000, NUMERATOR_16000, NUMERATOR_24000, NUMERATOR_32000, NUMERATOR_48000,
};
use super::{Pitch, gain_index};
use crate::config::{Config, FrameDuration, MAX_FRAME_SAMPLES, SampleRate};
use crate::math::nint;

/// Largest denominator order `L_den` (equation 148).
const MAX_DEN_ORDER: usize = 12;

/// Largest numerator order `L_num` (equation 149).
const MAX_NUM_ORDER: usize = MAX_DEN_ORDER - 2;

/// Largest integer pitch lag at the output rate, 228 at 12.8 kHz scaled to 48 kHz.
const MAX_LAG: usize = 228 * 48_000 / 12_800 + 1;

/// History of the output signal needed by the filter.
const OUTPUT_HISTORY: usize = MAX_LAG + MAX_DEN_ORDER / 2 + 2;

/// History of the input signal needed by the filter.
const INPUT_HISTORY: usize = MAX_DEN_ORDER;

/// Longest transition, 2.5 ms at 48 kHz.
const MAX_TRANSITION: usize = MAX_FRAME_SAMPLES / 4;

/// Gains `gain_ltpf` per gain index (section 3.4.9.4).
const GAINS: [f32; 4] = [0.4, 0.35, 0.3, 0.25];

/// Scaling of the numerator coefficients (equation 146).
const NUMERATOR_SCALE: f32 = 0.85;

/// Rate of the pitch lags of the bitstream, in Hz.
const PITCH_RATE: f32 = 12_800.0;

/// Coefficients of the postfilter (section 3.4.9.4).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Filter {
    /// Integer part of the pitch lag at the output rate (`p_int`).
    lag_int: usize,
    /// Fractional part of the pitch lag at the output rate, in quarters (`p_fr`).
    lag_fr: usize,
    /// Numerator coefficients (`c_num`).
    num: [f32; MAX_NUM_ORDER + 1],
    /// Denominator coefficients (`c_den`).
    den: [f32; MAX_DEN_ORDER + 1],
}

impl Filter {
    /// Coefficients of the postfilter of a pitch (section 3.4.9.4). The
    /// coefficients are zero when the bitrate is too high for the postfilter.
    fn new(config: &Config, pitch: Pitch, nbytes: usize) -> Self {
        // 44.1 kHz shares the pitch scaling and the tables of 48 kHz.
        let rate = match config.sample_rate {
            SampleRate::Hz44100 => 48_000,
            sample_rate => sample_rate.hz(),
        };
        let lag = pitch.index.lag() * rate as f32 / PITCH_RATE;
        let lag_quarters = nint(lag * 4.0);
        let mut filter = Self {
            lag_int: (lag_quarters / 4) as usize,
            lag_fr: (lag_quarters % 4) as usize,
            ..Self::default()
        };
        if let Some(gain_index) = gain_index(config, nbytes) {
            let gain = GAINS[gain_index];
            let (num, den): (&[f32], &[f32]) = match config.sample_rate {
                SampleRate::Hz8000 => (
                    &NUMERATOR_8000[gain_index],
                    &DENOMINATOR_8000[filter.lag_fr],
                ),
                SampleRate::Hz16000 => (
                    &NUMERATOR_16000[gain_index],
                    &DENOMINATOR_16000[filter.lag_fr],
                ),
                SampleRate::Hz24000 => (
                    &NUMERATOR_24000[gain_index],
                    &DENOMINATOR_24000[filter.lag_fr],
                ),
                SampleRate::Hz32000 => (
                    &NUMERATOR_32000[gain_index],
                    &DENOMINATOR_32000[filter.lag_fr],
                ),
                SampleRate::Hz44100 | SampleRate::Hz48000 => (
                    &NUMERATOR_48000[gain_index],
                    &DENOMINATOR_48000[filter.lag_fr],
                ),
            };
            for (c, &t) in filter.num.iter_mut().zip(num) {
                *c = NUMERATOR_SCALE * gain * t;
            }
            for (c, &t) in filter.den.iter_mut().zip(den) {
                *c = gain * t;
            }
        }
        filter
    }
}

/// Denominator order `L_den` of the postfilter (equation 148).
const fn den_order(sample_rate: SampleRate) -> usize {
    match sample_rate {
        SampleRate::Hz8000 | SampleRate::Hz16000 => 4,
        SampleRate::Hz24000 => 6,
        SampleRate::Hz32000 => 8,
        SampleRate::Hz44100 | SampleRate::Hz48000 => 12,
    }
}

/// State of the postfilter of the decoder.
#[derive(Clone)]
pub(crate) struct Synthesis {
    /// Filter of the previous frame, `None` when the postfilter was inactive.
    previous: Option<Filter>,
    /// Filter input `x^`, the current frame starting at `INPUT_HISTORY`.
    input: [f32; INPUT_HISTORY + MAX_FRAME_SAMPLES],
    /// Filter output `x^_ltpf`, the current frame starting at `OUTPUT_HISTORY`.
    output: [f32; OUTPUT_HISTORY + MAX_FRAME_SAMPLES],
    /// Intermediate output `x^'_ltpf` of the transition between two filters,
    /// the current frame starting at `INPUT_HISTORY`.
    intermediate: [f32; INPUT_HISTORY + MAX_TRANSITION],
}

impl Synthesis {
    /// Postfilter state before the first frame.
    pub(crate) const fn new() -> Self {
        Self {
            previous: None,
            input: [0.0; INPUT_HISTORY + MAX_FRAME_SAMPLES],
            output: [0.0; OUTPUT_HISTORY + MAX_FRAME_SAMPLES],
            intermediate: [0.0; INPUT_HISTORY + MAX_TRANSITION],
        }
    }

    /// Postfiltering of a frame in place (sections 3.4.9.2 and 3.4.9.3),
    /// `pitch` being the pitch of the frame when the postfilter is active.
    pub(crate) fn run(
        &mut self,
        config: &Config,
        pitch: Option<Pitch>,
        nbytes: usize,
        frame: &mut [f32],
    ) {
        let len = frame.len();
        let den_order = den_order(config.sample_rate);
        let num_order = den_order - 2;
        // Transition over the first 2.5 ms of the frame
        let transition = match config.frame_duration {
            FrameDuration::Ms10 => len / 4,
            FrameDuration::Ms7_5 => len / 3,
        };
        let current = pitch.map(|p| Filter::new(config, p, nbytes));

        self.input[INPUT_HISTORY..INPUT_HISTORY + len].copy_from_slice(frame);
        let input = &self.input;
        let output = &mut self.output;

        // Filter contribution at the sample n of the current frame:
        // sum c_num(k) x(n - k) - sum c_den(k) y(n - p_int + L_den/2 - k)
        let contribution = |filter: &Filter, x: &[f32], y: &[f32], n: usize| -> f32 {
            let mut sum = 0.0f32;
            for k in 0..=num_order {
                sum += filter.num[k] * x[INPUT_HISTORY + n - k];
            }
            let base = OUTPUT_HISTORY + n + den_order / 2 - filter.lag_int;
            for k in 0..=den_order {
                sum -= filter.den[k] * y[base - k];
            }
            sum
        };
        let fade_in = |n: usize| n as f32 / transition as f32;
        let fade_out = |n: usize| 1.0 - n as f32 / transition as f32;

        match (&self.previous, &current) {
            (None, None) => {
                output[OUTPUT_HISTORY..OUTPUT_HISTORY + transition]
                    .copy_from_slice(&input[INPUT_HISTORY..INPUT_HISTORY + transition]);
            }
            (None, Some(filter)) => {
                for n in 0..transition {
                    let c = contribution(filter, input, output, n);
                    output[OUTPUT_HISTORY + n] = input[INPUT_HISTORY + n] - fade_in(n) * c;
                }
            }
            (Some(previous), None) => {
                for n in 0..transition {
                    let c = contribution(previous, input, output, n);
                    output[OUTPUT_HISTORY + n] = input[INPUT_HISTORY + n] - fade_out(n) * c;
                }
            }
            (Some(previous), Some(filter))
                if previous.lag_int == filter.lag_int && previous.lag_fr == filter.lag_fr =>
            {
                for n in 0..transition {
                    let c = contribution(filter, input, output, n);
                    output[OUTPUT_HISTORY + n] = input[INPUT_HISTORY + n] - c;
                }
            }
            (Some(previous), Some(filter)) => {
                // Fade out of the previous filter, then fade in of the
                // current filter on the result (equations 134 to 136).
                for n in 0..transition {
                    let c = contribution(previous, input, output, n);
                    output[OUTPUT_HISTORY + n] = input[INPUT_HISTORY + n] - fade_out(n) * c;
                }
                let intermediate = &mut self.intermediate;
                intermediate[INPUT_HISTORY - num_order..INPUT_HISTORY + transition]
                    .copy_from_slice(
                        &output[OUTPUT_HISTORY - num_order..OUTPUT_HISTORY + transition],
                    );
                for n in 0..transition {
                    let c = contribution(filter, intermediate, output, n);
                    output[OUTPUT_HISTORY + n] = intermediate[INPUT_HISTORY + n] - fade_in(n) * c;
                }
            }
        }

        // Remainder of the frame (equations 137 and 138)
        match &current {
            Some(filter) => {
                for n in transition..len {
                    let c = contribution(filter, input, output, n);
                    output[OUTPUT_HISTORY + n] = input[INPUT_HISTORY + n] - c;
                }
            }
            None => {
                output[OUTPUT_HISTORY + transition..OUTPUT_HISTORY + len]
                    .copy_from_slice(&input[INPUT_HISTORY + transition..INPUT_HISTORY + len]);
            }
        }

        frame.copy_from_slice(&output[OUTPUT_HISTORY..OUTPUT_HISTORY + len]);
        self.input.copy_within(len..len + INPUT_HISTORY, 0);
        self.output.copy_within(len..len + OUTPUT_HISTORY, 0);
        self.previous = current;
    }
}

#[cfg(test)]
mod tests {
    use super::{Filter, INPUT_HISTORY, OUTPUT_HISTORY, Synthesis};
    use crate::config::{Config, FrameDuration, SampleRate};
    use crate::ltpf::{Pitch, PitchIndex};
    use crate::test_vectors::{
        LTPF_CASE_2_INPUT, LTPF_CASE_2_OUTPUT, LTPF_CASE_2_PREVIOUS_INPUT,
        LTPF_CASE_2_PREVIOUS_OUTPUTS, LTPF_CASE_3_INPUT, LTPF_CASE_3_OUTPUT,
        LTPF_CASE_3_PREVIOUS_INPUT, LTPF_CASE_3_PREVIOUS_OUTPUTS, LTPF_CASE_4_INPUT,
        LTPF_CASE_4_OUTPUT, LTPF_CASE_4_PREVIOUS_INPUT, LTPF_CASE_4_PREVIOUS_OUTPUTS,
        LTPF_CASE_5_INPUT, LTPF_CASE_5_OUTPUT, LTPF_CASE_5_PREVIOUS_INPUT,
        LTPF_CASE_5_PREVIOUS_OUTPUTS,
    };

    /// Configuration of the vectors of appendix C.4.5, 16 kHz at 10 ms.
    const CONFIG: Config = Config::new(SampleRate::Hz16000, FrameDuration::Ms10);

    /// Size of the frames of appendix C.4.5, 320 bits.
    const NBYTES: usize = 40;

    /// Active pitch of a pitch index.
    fn pitch(index: u16) -> Pitch {
        Pitch {
            index: PitchIndex(index),
            active: true,
        }
    }

    /// Runs a transition of appendix C.4.5 and checks its output.
    fn check_transition(
        previous: Option<Pitch>,
        current: Option<Pitch>,
        previous_input: &[f64; 160],
        previous_outputs: &[f64; 320],
        input: &[f64; 160],
        expected: &[f64; 160],
    ) {
        let mut synthesis = Synthesis::new();
        synthesis.previous = previous.map(|p| Filter::new(&CONFIG, p, NBYTES));
        for (s, &v) in synthesis.input[..INPUT_HISTORY]
            .iter_mut()
            .zip(&previous_input[160 - INPUT_HISTORY..])
        {
            *s = v as f32;
        }
        for (s, &v) in synthesis.output[OUTPUT_HISTORY - 320..OUTPUT_HISTORY]
            .iter_mut()
            .zip(previous_outputs)
        {
            *s = v as f32;
        }

        let mut frame = input.map(|v| v as f32);
        synthesis.run(&CONFIG, current, NBYTES, &mut frame);
        for (n, (&got, &want)) in frame.iter().zip(expected).enumerate() {
            assert!(
                (f64::from(got) - want).abs() < 0.01,
                "[{n}]: got {got}, want {want}"
            );
        }
    }

    #[test]
    fn transition_from_inactive_to_active() {
        check_transition(
            None,
            Some(pitch(56)),
            &LTPF_CASE_2_PREVIOUS_INPUT,
            &LTPF_CASE_2_PREVIOUS_OUTPUTS,
            &LTPF_CASE_2_INPUT,
            &LTPF_CASE_2_OUTPUT,
        );
    }

    #[test]
    fn transition_from_active_to_inactive() {
        check_transition(
            Some(pitch(56)),
            None,
            &LTPF_CASE_3_PREVIOUS_INPUT,
            &LTPF_CASE_3_PREVIOUS_OUTPUTS,
            &LTPF_CASE_3_INPUT,
            &LTPF_CASE_3_OUTPUT,
        );
    }

    #[test]
    fn active_with_the_same_pitch() {
        check_transition(
            Some(pitch(56)),
            Some(pitch(56)),
            &LTPF_CASE_4_PREVIOUS_INPUT,
            &LTPF_CASE_4_PREVIOUS_OUTPUTS,
            &LTPF_CASE_4_INPUT,
            &LTPF_CASE_4_OUTPUT,
        );
    }

    #[test]
    fn transition_between_two_pitches() {
        check_transition(
            Some(pitch(56)),
            Some(pitch(52)),
            &LTPF_CASE_5_PREVIOUS_INPUT,
            &LTPF_CASE_5_PREVIOUS_OUTPUTS,
            &LTPF_CASE_5_INPUT,
            &LTPF_CASE_5_OUTPUT,
        );
    }
}
