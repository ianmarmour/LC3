//! LTPF encoder: pitch analysis and activation decision (section 3.3.9).

use super::tables::{INTERPOLATION_R, INTERPOLATION_X12K8, RESAMPLING_FILTER};
use super::{MIN_PITCH, Pitch, PitchIndex, gain_index};
use crate::config::{Config, FrameDuration, SampleRate};

/// Smallest lag of the pitch search at 6.4 kHz.
const K_MIN: usize = 17;

/// Largest lag of the pitch search at 6.4 kHz.
const K_MAX: usize = 114;

/// Largest integer pitch lag at 12.8 kHz.
const MAX_PITCH: usize = 228;

/// Half width of the search of the pitch at 12.8 kHz around twice the pitch
/// at 6.4 kHz, and of the interpolation of the autocorrelation.
const SEARCH_RADIUS: usize = 4;

/// Half width of the search of the pitch around the pitch of the previous frame.
const TRACKING_RADIUS: usize = 4;

/// Normalized correlation of the pitch above which the pitch is transmitted
/// (equation 94).
const PITCH_PRESENT_CORRELATION: f32 = 0.6;

/// Ratio of normalized correlations favoring the pitch of the previous frame
/// (equation 91).
const TRACKING_RATIO: f32 = 0.85;

/// Normalized correlation above which the postfilter is activated
/// (section 3.3.9.8).
const ACTIVATION_CORRELATION: f32 = 0.94;

/// Normalized correlation above which an active postfilter stays active.
const SUSTAIN_CORRELATION: f32 = 0.9;

/// Normalized correlation above which an active postfilter stays active when
/// the pitch is stable.
const STABLE_PITCH_CORRELATION: f32 = 0.84;

/// Largest pitch change of a stable pitch.
const STABLE_PITCH_CHANGE: f32 = 2.0;

/// Largest drop of the normalized correlation of a stable pitch.
const STABLE_CORRELATION_DROP: f32 = 0.1;

/// Half width of the resampling filter, at 192 kHz.
const RESAMPLING_SPAN: usize = 120;

/// Decimation of the 192 kHz signal to 12.8 kHz.
const DECIMATION: usize = 15;

/// History of the 12.8 kHz signal needed by the analysis: the largest lag of
/// the pitch search and the span of the interpolation filter.
const HISTORY_12K8: usize = MAX_PITCH + SEARCH_RADIUS + 2;

/// Size of the buffer of the 12.8 kHz signal: history, largest delay and
/// largest frame.
const BUFFER_12K8: usize = HISTORY_12K8 + 44 + 128;

/// Size of the buffer of the 6.4 kHz signal: history and largest frame.
const BUFFER_6K4: usize = K_MAX + 64;

/// Low-pass filter of the downsampling to 6.4 kHz (equation 85).
#[allow(clippy::excessive_precision)]
const DOWNSAMPLING_FILTER: [f32; 5] = [
    0.1236796411180537, 0.2353512128364889, 0.2819382920909148, 0.2353512128364889,
    0.1236796411180537,
];

/// 50 Hz high-pass filter `H50(z)` (equation 83), numerator `b0, b1, b2`.
#[allow(clippy::excessive_precision)]
const HIGH_PASS_B: [f32; 3] = [0.9827947082978771, -1.965589416595754, 0.9827947082978771];

/// 50 Hz high-pass filter `H50(z)` (equation 83), denominator `a1, a2`.
#[allow(clippy::excessive_precision)]
const HIGH_PASS_A: [f32; 2] = [-1.9652933726226904, 0.9658854605688177];

/// Upsampling factor `P` of the resampling to 12.8 kHz, `192 kHz / f_s`, and
/// gain `resfac` of the resampling (equations 78 and 82). 44.1 kHz is
/// resampled as 48 kHz, to 11.76 kHz.
const fn resampling(sample_rate: SampleRate) -> (usize, f32) {
    match sample_rate {
        SampleRate::Hz8000 => (24, 0.5),
        SampleRate::Hz16000 => (12, 1.0),
        SampleRate::Hz24000 => (8, 1.0),
        SampleRate::Hz32000 => (6, 1.0),
        SampleRate::Hz44100 | SampleRate::Hz48000 => (4, 1.0),
    }
}

/// Number of samples of a frame at 12.8 kHz (`len12.8`, equation 80).
const fn frame_len_12k8(frame_duration: FrameDuration) -> usize {
    match frame_duration {
        FrameDuration::Ms10 => 128,
        FrameDuration::Ms7_5 => 96,
    }
}

/// Delay `D_LTPF` of the 12.8 kHz signal.
const fn delay(frame_duration: FrameDuration) -> usize {
    match frame_duration {
        FrameDuration::Ms10 => 24,
        FrameDuration::Ms7_5 => 44,
    }
}

/// Normalized correlation of `x` at the lag `lag` over `len` samples from
/// `start` (equation 92).
fn normalized_correlation(x: &[f32], start: usize, len: usize, lag: usize) -> f32 {
    let (mut corr, mut energy0, mut energy1) = (0.0f32, 0.0f32, 0.0f32);
    for n in start..start + len {
        let (a, b) = (x[n], x[n - lag]);
        corr += a * b;
        energy0 += a * a;
        energy1 += b * b;
    }
    let norm = libm::sqrtf(energy0 * energy1);
    if norm > 0.0 {
        f32::max(0.0, corr / norm)
    } else {
        0.0
    }
}

/// State of the LTPF analysis of the encoder.
#[derive(Clone)]
pub(crate) struct Analysis {
    /// State of the high-pass filter (direct form II transposed).
    high_pass: [f32; 2],
    /// High-pass filtered signal at 12.8 kHz `x~12.8`, the current frame
    /// starting at `HISTORY_12K8 + D_LTPF`.
    x12k8: [f32; BUFFER_12K8],
    /// Signal at 6.4 kHz, the current frame starting at `K_MAX`.
    x6k4: [f32; BUFFER_6K4],
    /// Pitch lag at 6.4 kHz of the previous frame (`T_prev`).
    previous_lag: usize,
    /// Pitch lag of the previous frame, 0 when not transmitted (`mem_pitch`).
    previous_pitch: f32,
    /// Whether the postfilter was active in the previous frame (`mem_ltpf_active`).
    previous_active: bool,
    /// Normalized correlation of the previous frame, 0 when the pitch was not
    /// transmitted (`mem_nc`).
    previous_nc: f32,
    /// Normalized correlation of the penultimate frame (`mem_mem_nc`).
    penultimate_nc: f32,
}

impl Analysis {
    /// Analysis state before the first frame.
    pub(crate) const fn new() -> Self {
        Self {
            high_pass: [0.0; 2],
            x12k8: [0.0; BUFFER_12K8],
            x6k4: [0.0; BUFFER_6K4],
            previous_lag: K_MIN,
            previous_pitch: 0.0,
            previous_active: false,
            previous_nc: 0.0,
            penultimate_nc: 0.0,
        }
    }

    /// Delayed high-pass filtered 12.8 kHz signal `x~12.8_D(n)` of the current frame.
    fn delayed(&self, n: isize) -> f32 {
        self.x12k8[(HISTORY_12K8 as isize + n) as usize]
    }

    /// Resampling of a frame to 12.8 kHz (equation 78) and high-pass
    /// filtering (equation 83). `xs` holds the input signal, the frame
    /// starting at `start` after at least 60 samples of history.
    fn resample(&mut self, config: &Config, xs: &[f32], start: usize) {
        let (upsampling, gain) = resampling(config.sample_rate);
        let half = (RESAMPLING_SPAN / upsampling) as isize;
        let current = HISTORY_12K8 + delay(config.frame_duration);
        for n in 0..frame_len_12k8(config.frame_duration) {
            let base = start as isize + (DECIMATION * n / upsampling) as isize - half;
            let phase = (DECIMATION * n % upsampling) as isize;
            let mut acc = 0.0f32;
            for k in -half..=half {
                let tap = upsampling as isize * k - phase;
                if tap.unsigned_abs() >= RESAMPLING_SPAN {
                    continue;
                }
                acc += xs[(base + k) as usize]
                    * RESAMPLING_FILTER[(tap + RESAMPLING_SPAN as isize - 1) as usize];
            }
            let x = acc * gain * upsampling as f32;

            let y = HIGH_PASS_B[0] * x + self.high_pass[0];
            self.high_pass[0] = HIGH_PASS_B[1] * x - HIGH_PASS_A[0] * y + self.high_pass[1];
            self.high_pass[1] = HIGH_PASS_B[2] * x - HIGH_PASS_A[1] * y;
            self.x12k8[current + n] = y;
        }
    }

    /// Pitch lags at 6.4 kHz maximizing the weighted and the unweighted
    /// autocorrelation (equations 86 to 90).
    fn pitch_candidates(&self, len: usize) -> (usize, usize) {
        let mut r = [0.0f32; K_MAX + 1];
        for (k, rk) in r.iter_mut().enumerate().skip(K_MIN) {
            let mut acc = 0.0f32;
            for n in K_MAX..K_MAX + len {
                acc += self.x6k4[n] * self.x6k4[n - k];
            }
            *rk = acc;
        }
        let mut weighted_best = K_MIN;
        let mut best = f32::NEG_INFINITY;
        for (k, &rk) in r.iter().enumerate().skip(K_MIN) {
            let weight = 1.0 - 0.5 * (k - K_MIN) as f32 / (K_MAX - K_MIN) as f32;
            if rk * weight > best {
                best = rk * weight;
                weighted_best = k;
            }
        }
        let low = K_MIN.max(self.previous_lag.saturating_sub(TRACKING_RADIUS));
        let high = K_MAX.min(self.previous_lag + TRACKING_RADIUS);
        let mut tracked_best = low;
        for k in low..=high {
            if r[k] > r[tracked_best] {
                tracked_best = k;
            }
        }
        (weighted_best, tracked_best)
    }

    /// Integer and fractional parts of the pitch lag at 12.8 kHz around
    /// twice the pitch lag `lag` at 6.4 kHz (equations 96 to 101).
    fn pitch_lag(&self, len: usize, lag: usize) -> (i32, i32) {
        let low = (MIN_PITCH as usize).max(2 * lag - SEARCH_RADIUS);
        let high = MAX_PITCH.min(2 * lag + SEARCH_RADIUS);
        let first = low as isize - SEARCH_RADIUS as isize;
        let mut r = [0.0f32; 4 * SEARCH_RADIUS + 1];
        for (i, rv) in r
            .iter_mut()
            .enumerate()
            .take(high - low + 1 + 2 * SEARCH_RADIUS)
        {
            let k = first + i as isize;
            let mut acc = 0.0f32;
            for n in 0..len as isize {
                acc += self.delayed(n) * self.delayed(n - k);
            }
            *rv = acc;
        }
        let r_at = |k: isize| r[(k - first) as usize];

        let mut pitch_int = low;
        for k in low..=high {
            if r_at(k as isize) > r_at(pitch_int as isize) {
                pitch_int = k;
            }
        }
        let interpolated = |d: isize| -> f32 {
            let mut acc = 0.0f32;
            for m in -(SEARCH_RADIUS as isize)..=SEARCH_RADIUS as isize {
                let tap = 4 * m - d;
                let half = INTERPOLATION_R.len() / 2;
                if tap.unsigned_abs() <= half {
                    acc += r_at(pitch_int as isize + m)
                        * INTERPOLATION_R[(tap + half as isize) as usize];
                }
            }
            acc
        };
        let pitch_int = pitch_int as i32;
        let candidates: &[isize] = if pitch_int >= super::INTEGER_RESOLUTION_PITCH {
            &[]
        } else if pitch_int >= super::HALF_RESOLUTION_PITCH {
            &[-2, 0, 2]
        } else if pitch_int > MIN_PITCH {
            &[-3, -2, -1, 0, 1, 2, 3]
        } else {
            &[0, 1, 2, 3]
        };
        let mut pitch_fr = 0;
        let mut best = f32::NEG_INFINITY;
        for &d in candidates {
            let v = interpolated(d);
            if v > best {
                best = v;
                pitch_fr = d as i32;
            }
        }
        if pitch_fr < 0 {
            (pitch_int - 1, pitch_fr + 4)
        } else {
            (pitch_int, pitch_fr)
        }
    }

    /// Normalized correlation at the pitch lag of the interpolated 12.8 kHz
    /// signal (equations 103 to 105).
    fn pitch_correlation(&self, len: usize, pitch_int: i32, pitch_fr: i32) -> f32 {
        let interpolated = |n: isize, d: isize| -> f32 {
            let mut acc = 0.0f32;
            for k in -2isize..=2 {
                let tap = 4 * k - d;
                let half = INTERPOLATION_X12K8.len() / 2;
                if tap.unsigned_abs() <= half {
                    acc +=
                        self.delayed(n - k) * INTERPOLATION_X12K8[(tap + half as isize) as usize];
                }
            }
            acc
        };
        let (mut corr, mut energy0, mut energy1) = (0.0f32, 0.0f32, 0.0f32);
        for n in 0..len as isize {
            let a = interpolated(n, 0);
            let b = interpolated(n - pitch_int as isize, pitch_fr as isize);
            corr += a * b;
            energy0 += a * a;
            energy1 += b * b;
        }
        let norm = libm::sqrtf(energy0 * energy1);
        if norm > 0.0 { corr / norm } else { 0.0 }
    }

    /// Activation of the postfilter (section 3.3.9.8).
    fn activation(&self, config: &Config, nc: f32, pitch: f32) -> bool {
        let onset = config.frame_duration == FrameDuration::Ms10
            || self.penultimate_nc > ACTIVATION_CORRELATION;
        (!self.previous_active
            && onset
            && self.previous_nc > ACTIVATION_CORRELATION
            && nc > ACTIVATION_CORRELATION)
            || (self.previous_active && nc > SUSTAIN_CORRELATION)
            || (self.previous_active
                && libm::fabsf(pitch - self.previous_pitch) < STABLE_PITCH_CHANGE
                && (nc - self.previous_nc) > -STABLE_CORRELATION_DROP
                && nc > STABLE_PITCH_CORRELATION)
    }

    /// Analysis of a frame. `xs` holds the scaled input signal, the frame
    /// starting at `start` after at least 60 samples of history.
    pub(crate) fn run(
        &mut self,
        config: &Config,
        xs: &[f32],
        start: usize,
        nbytes: usize,
        near_nyquist: bool,
    ) -> Option<Pitch> {
        let len = frame_len_12k8(config.frame_duration);
        let len_6k4 = len / 2;
        self.x12k8.copy_within(len.., 0);
        self.x6k4.copy_within(len_6k4.., 0);
        self.resample(config, xs, start);

        // Downsampling to 6.4 kHz (equation 85)
        for n in 0..len_6k4 {
            let mut acc = 0.0f32;
            for (k, h) in DOWNSAMPLING_FILTER.iter().enumerate() {
                acc += self.delayed(2 * n as isize + k as isize - 3) * h;
            }
            self.x6k4[K_MAX + n] = acc;
        }

        // Pitch estimation (equation 91) and presence (equation 94)
        let (weighted_lag, tracked_lag) = self.pitch_candidates(len_6k4);
        let weighted_nc = normalized_correlation(&self.x6k4, K_MAX, len_6k4, weighted_lag);
        let tracked_nc = normalized_correlation(&self.x6k4, K_MAX, len_6k4, tracked_lag);
        let (lag, lag_nc) = if tracked_nc <= TRACKING_RATIO * weighted_nc {
            (weighted_lag, weighted_nc)
        } else {
            (tracked_lag, tracked_nc)
        };
        self.previous_lag = lag;
        if lag_nc <= PITCH_PRESENT_CORRELATION {
            self.penultimate_nc = self.previous_nc;
            self.previous_nc = 0.0;
            self.previous_pitch = 0.0;
            self.previous_active = false;
            return None;
        }

        let (pitch_int, pitch_fr) = self.pitch_lag(len, lag);
        let nc = self.pitch_correlation(len, pitch_int, pitch_fr);
        let pitch = pitch_int as f32 + pitch_fr as f32 / 4.0;
        let active = gain_index(config, nbytes).is_some()
            && !near_nyquist
            && self.activation(config, nc, pitch);

        self.penultimate_nc = self.previous_nc;
        self.previous_nc = nc;
        self.previous_pitch = pitch;
        self.previous_active = active;
        Some(Pitch {
            index: PitchIndex::from_lag(pitch_int, pitch_fr),
            active,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::Analysis;
    use crate::config::{Config, FrameDuration, SampleRate};

    /// A pulse train of period 80 samples at 16 kHz has a pitch lag of 64
    /// samples at 12.8 kHz.
    #[test]
    fn detects_the_pitch_of_a_pulse_train() {
        let config = Config::new(SampleRate::Hz16000, FrameDuration::Ms10);
        let history = 100;
        let mut analysis = Analysis::new();
        let mut pitch = None;
        for frame in 0..10 {
            let xs: Vec<f32> = (0..history + 160)
                .map(|n| {
                    if (frame * 160 + n) % 80 == 0 {
                        10_000.0
                    } else {
                        0.0
                    }
                })
                .collect();
            pitch = analysis.run(&config, &xs, history, 40, false);
        }
        let pitch = pitch.expect("pitch of a periodic signal");
        assert!(
            (pitch.index.lag() - 64.0).abs() <= 0.5,
            "lag {}",
            pitch.index.lag()
        );
    }
}
