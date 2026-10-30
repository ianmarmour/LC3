//! Bandwidth detector (section 3.3.5).

use crate::config::{Bandwidth, Config, FrameDuration};

/// Mean band energy below which a frequency region is quiet, per bandwidth
/// (`T_Q`).
const QUIET_THRESHOLDS: [f32; 4] = [20.0, 10.0, 10.0, 10.0];

/// Energy drop at the cut-off frequency, in dB, detecting a band limited
/// signal, per bandwidth (`T_C`).
const DROP_THRESHOLDS: [f32; 4] = [15.0, 23.0, 20.0, 20.0];

/// Offset avoiding the logarithm of zero (equation 13).
const LOG_OFFSET: f32 = 1e-31;

/// Band ranges above the cut-off frequency of each bandwidth, the first
/// and last bands `I_bw_start` and `I_bw_stop` (table 3.6).
const fn regions(config: &Config) -> [(usize, usize); 4] {
    match (config.frame_duration, config.sample_rate.index()) {
        (FrameDuration::Ms10, 1) => [(53, 63), (0, 0), (0, 0), (0, 0)],
        (FrameDuration::Ms10, 2) => [(47, 56), (59, 63), (0, 0), (0, 0)],
        (FrameDuration::Ms10, 3) => [(44, 52), (54, 59), (60, 63), (0, 0)],
        (FrameDuration::Ms10, _) => [(41, 49), (51, 55), (57, 60), (61, 63)],
        (FrameDuration::Ms7_5, 1) => [(51, 63), (0, 0), (0, 0), (0, 0)],
        (FrameDuration::Ms7_5, 2) => [(45, 55), (58, 63), (0, 0), (0, 0)],
        (FrameDuration::Ms7_5, 3) => [(42, 51), (53, 58), (60, 63), (0, 0)],
        (FrameDuration::Ms7_5, _) => [(40, 48), (51, 55), (57, 60), (61, 63)],
    }
}

/// Distance in bands of the energy drop detection, per bandwidth (`L`).
const fn drop_distances(frame_duration: FrameDuration) -> [usize; 4] {
    match frame_duration {
        FrameDuration::Ms10 => [4, 4, 3, 1],
        FrameDuration::Ms7_5 => [4, 4, 3, 2],
    }
}

/// Energy ratio in dB between two bands, a band without energy being
/// infinitely below any band with energy.
fn energy_ratio_db(numerator: f32, denominator: f32) -> f32 {
    if denominator > 0.0 {
        10.0 * libm::log10f(LOG_OFFSET + numerator / denominator)
    } else if numerator > 0.0 {
        f32::INFINITY
    } else {
        10.0 * libm::log10f(LOG_OFFSET)
    }
}

/// Bandwidth of a frame from its band energies `eb` (section 3.3.5.1).
pub(super) fn detect(config: &Config, eb: &[f32]) -> Bandwidth {
    let widest = config.max_bandwidth();
    let candidates = widest.index();
    if candidates == 0 {
        return widest;
    }
    let regions = regions(config);

    // First stage: the narrowest bandwidth above which all regions are quiet
    let quiet = |k: usize| {
        let (start, stop) = regions[k];
        let mean = eb[start..=stop].iter().sum::<f32>() / (stop - start + 1) as f32;
        mean < QUIET_THRESHOLDS[k]
    };
    let mut bw0 = candidates;
    while bw0 > 0 && quiet(bw0 - 1) {
        bw0 -= 1;
    }
    if bw0 == candidates {
        return widest;
    }

    // Second stage: an energy drop above the cut-off frequency (equation 13)
    let distance = drop_distances(config.frame_duration)[bw0];
    let start = regions[bw0].0;
    let drop = (start + 1 - distance..=start + 1)
        .map(|n| energy_ratio_db(eb[n - distance], eb[n]))
        .fold(f32::NEG_INFINITY, f32::max);
    if drop > DROP_THRESHOLDS[bw0] {
        Bandwidth::from_index(bw0).unwrap_or(widest)
    } else {
        widest
    }
}

#[cfg(test)]
mod tests {
    use super::detect;
    use crate::config::{Bandwidth, Config, FrameDuration, SampleRate};

    /// Band energies of a signal limited to the bands below `cutoff`.
    fn energies(cutoff: usize) -> [f32; 64] {
        core::array::from_fn(|b| if b < cutoff { 1e6 } else { 1e-3 })
    }

    #[test]
    fn detects_band_limited_signals() {
        let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
        assert_eq!(detect(&config, &energies(64)), Bandwidth::Full);
        assert_eq!(detect(&config, &energies(61)), Bandwidth::SuperWide);
        assert_eq!(detect(&config, &energies(41)), Bandwidth::Narrow);
    }

    #[test]
    fn narrow_band_at_8_khz() {
        let config = Config::new(SampleRate::Hz8000, FrameDuration::Ms10);
        assert_eq!(detect(&config, &energies(64)), Bandwidth::Narrow);
    }
}
