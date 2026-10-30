//! Two-stage vector quantization of the scale factors (sections 3.3.7.3 and
//! 3.4.7.2): a split VQ followed by a pyramid VQ of the residual in a rotated
//! domain.

use super::SCALE_FACTORS;
use super::tables::{
    D, HFCB, LFCB, MPVQ_OFFSETS, OUTLIER_FAR_GAINS, OUTLIER_NEAR_GAINS, REGULAR_GAINS,
    REGULAR_LF_GAINS,
};
use crate::bitstream::{Reader, Writer};

/// Dimension of each split of the first stage.
const SPLIT: usize = 8;

/// Number of bits of each first stage index.
const STAGE1_BITS: u32 = 5;

/// Number of positions of the set A of the regular shapes.
const SET_A: usize = 10;

/// Number of unit pulses of the regular shapes over the set A.
const REGULAR_PULSES: i32 = 10;

/// Number of unit pulses of the near outlier shape.
const OUTLIER_NEAR_PULSES: i32 = 8;

/// Number of unit pulses of the far outlier shape.
const OUTLIER_FAR_PULSES: i32 = 6;

/// Number of MPVQ shape indices of the set A of the regular shapes (table 3.14).
const SIZE_REGULAR_A: u32 = 2_390_004;

/// Number of MPVQ shape indices of the near outlier shape (table 3.14).
const SIZE_OUTLIER_NEAR: u32 = 15_158_272;

/// Number of MPVQ shape indices of the far outlier shape (table 3.14).
const SIZE_OUTLIER_FAR: u32 = 774_912;

/// Number of values of the joint index of the regular shapes: the
/// regular low frequency shape takes 2 slots (its gain LSB), the regular
/// shape 12 (its 6 set B positions with 2 signs).
const REGULAR_JOINT_SLOTS: u32 = 14;

/// Number of bits of the low part of the joint index of the regular shapes.
const REGULAR_JOINT_LOW_BITS: u32 = 13;

/// Number of bits of the low part of the joint index of the outlier shapes.
const OUTLIER_JOINT_LOW_BITS: u32 = 12;

/// Number of bits of the high part of the joint index, for all shapes.
const JOINT_HIGH_BITS: u32 = 12;

/// Shape of the second stage (table 3.8).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum Shape {
    /// 10 pulses over the set A and 1 pulse over the set B (`regular`).
    #[default]
    Regular,
    /// 10 pulses over the set A only (`regular_lf`).
    RegularLf,
    /// 8 pulses over the 16 positions (`outlier_near`).
    OutlierNear,
    /// 6 pulses over the 16 positions (`outlier_far`).
    OutlierFar,
}

impl Shape {
    /// All the shapes, by shape index `shape_j`.
    const ALL: [Self; 4] = [
        Self::Regular,
        Self::RegularLf,
        Self::OutlierNear,
        Self::OutlierFar,
    ];

    /// Shape of the submode bits.
    fn from_submodes(msb: u32, lsb: u32) -> Self {
        Self::ALL[((msb << 1) | lsb) as usize]
    }

    /// Submode bit transmitted with the side information, 1 for the outliers.
    fn submode_msb(self) -> u32 {
        matches!(self, Self::OutlierNear | Self::OutlierFar).into()
    }

    /// Adjustment gains of the shape (table 3.11).
    fn gains(self) -> &'static [f32] {
        match self {
            Self::Regular => &REGULAR_GAINS,
            Self::RegularLf => &REGULAR_LF_GAINS,
            Self::OutlierNear => &OUTLIER_NEAR_GAINS,
            Self::OutlierFar => &OUTLIER_FAR_GAINS,
        }
    }

    /// Number of bits of the gain index sent with the side information
    /// (`sns_gainMSBbits`).
    fn gain_msb_bits(self) -> u32 {
        match self {
            Self::Regular | Self::RegularLf => 1,
            Self::OutlierNear | Self::OutlierFar => 2,
        }
    }

    /// Number of bits of the gain index sent within the joint index
    /// (`sns_gainLSBbits`).
    fn gain_lsb_bits(self) -> u32 {
        match self {
            Self::Regular | Self::OutlierNear => 0,
            Self::RegularLf | Self::OutlierFar => 1,
        }
    }
}

/// Quantization indices of the scale factors.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Indices {
    /// First stage index of the low frequency split (`ind_LF`).
    pub(crate) ind_lf: usize,
    /// First stage index of the high frequency split (`ind_HF`).
    pub(crate) ind_hf: usize,
    /// Second stage shape (`shape_j`).
    pub(crate) shape: Shape,
    /// Adjustment gain index (`gain_i`).
    pub(crate) gain: usize,
    /// Leading sign index of the set A, 1 when negative (`LS_indA`).
    pub(crate) ls_a: u32,
    /// MPVQ shape index of the set A (`idxA`).
    pub(crate) idx_a: u32,
    /// Leading sign index of the set B, 1 when negative (`LS_indB`).
    pub(crate) ls_b: u32,
    /// MPVQ shape index of the set B (`idxB`).
    pub(crate) idx_b: u32,
}

/// Integer pulse vectors `y_j` of the four shapes, signed.
type Pulses = [[i32; SCALE_FACTORS]; 4];

/// Index of the codebook vector nearest to `target` (equations 35 to 38).
fn nearest(codebook: &[[f32; SPLIT]; 32], target: &[f32]) -> usize {
    let mut best = 0;
    let mut best_distortion = f32::INFINITY;
    for (i, row) in codebook.iter().enumerate() {
        let distortion: f32 = row.iter().zip(target).map(|(c, t)| (t - c) * (t - c)).sum();
        if distortion < best_distortion {
            best_distortion = distortion;
            best = i;
        }
    }
    best
}

/// First stage vector `st1` of the first stage indices (equations 39 and 40).
fn stage1_vector(ind_lf: usize, ind_hf: usize) -> [f32; SCALE_FACTORS] {
    let mut st1 = [0.0; SCALE_FACTORS];
    st1[..SPLIT].copy_from_slice(&LFCB[ind_lf]);
    st1[SPLIT..].copy_from_slice(&HFCB[ind_hf]);
    st1
}

/// Second stage target: the first stage residual rotated by `D` (equation 43).
fn rotate(r1: &[f32; SCALE_FACTORS]) -> [f32; SCALE_FACTORS] {
    let mut t2rot = [0.0; SCALE_FACTORS];
    for (n, t) in t2rot.iter_mut().enumerate() {
        *t = (0..SCALE_FACTORS).map(|row| r1[row] * D[row][n]).sum();
    }
    t2rot
}

/// Adds unit pulses to `y` at the positions `positions` until their L1 norm
/// reaches `pulses`, maximizing the normalized correlation with the target of
/// absolute values `ax` (equations 48 to 52).
fn add_pulses(
    ax: &[f32; SCALE_FACTORS],
    y: &mut [i32; SCALE_FACTORS],
    positions: core::ops::Range<usize>,
    pulses: i32,
    corr: &mut f32,
    energy: &mut f32,
) {
    let mut placed: i32 = y[positions.clone()].iter().sum();
    while placed < pulses {
        let mut best = positions.start;
        let mut best_corr_sq = 0.0f32;
        let mut best_energy = 0.0f32;
        for n in positions.clone() {
            let c = *corr + ax[n];
            let e = *energy + 2.0 * y[n] as f32 + 1.0;
            let c2 = c * c;
            if n == positions.start || c2 * best_energy > best_corr_sq * e {
                best = n;
                best_corr_sq = c2;
                best_energy = e;
            }
        }
        *corr += ax[best];
        *energy += 2.0 * y[best] as f32 + 1.0;
        y[best] += 1;
        placed += 1;
    }
}

/// Correlation of the target of absolute values `ax` with the pulses `y`, and
/// energy of the pulses, over `positions`.
fn correlation_and_energy(
    ax: &[f32; SCALE_FACTORS],
    y: &[i32; SCALE_FACTORS],
    positions: core::ops::Range<usize>,
) -> (f32, f32) {
    let corr = positions.clone().map(|n| ax[n] * y[n] as f32).sum();
    let energy = y[positions].iter().map(|&v| (v * v) as f32).sum();
    (corr, energy)
}

/// Pulse vectors of the four shapes for the target `t2rot`, following the
/// search strategy of table 3.9.
fn search_shapes(t2rot: &[f32; SCALE_FACTORS]) -> Pulses {
    let ax = t2rot.map(libm::fabsf);
    let mut y = [0i32; SCALE_FACTORS];

    // Projection to or below the pyramid of the far outlier shape (equations 53 and 54)
    let sum_abs: f32 = ax.iter().sum();
    if sum_abs > 0.0 {
        let projection = (OUTLIER_FAR_PULSES - 1) as f32 / sum_abs;
        for (yn, &a) in y.iter_mut().zip(&ax) {
            *yn = libm::floorf(a * projection) as i32;
        }
    }
    let (mut corr, mut energy) = correlation_and_energy(&ax, &y, 0..SCALE_FACTORS);
    add_pulses(
        &ax,
        &mut y,
        0..SCALE_FACTORS,
        OUTLIER_FAR_PULSES,
        &mut corr,
        &mut energy,
    );
    let far = y;
    add_pulses(
        &ax,
        &mut y,
        0..SCALE_FACTORS,
        OUTLIER_NEAR_PULSES,
        &mut corr,
        &mut energy,
    );
    let near = y;

    // The regular shapes keep the pulses of the set A only.
    y[SET_A..].fill(0);
    let (mut corr, mut energy) = correlation_and_energy(&ax, &y, 0..SET_A);
    add_pulses(
        &ax,
        &mut y,
        0..SET_A,
        REGULAR_PULSES,
        &mut corr,
        &mut energy,
    );
    let regular_lf = y;
    let mut set_b_max = SET_A;
    for n in SET_A + 1..SCALE_FACTORS {
        if ax[n] > ax[set_b_max] {
            set_b_max = n;
        }
    }
    y[set_b_max] += 1;
    let regular = y;

    let mut pulses = [regular, regular_lf, near, far];
    for yj in &mut pulses {
        for (v, &t) in yj.iter_mut().zip(t2rot) {
            if t < 0.0 {
                *v = -*v;
            }
        }
    }
    pulses
}

/// Pulse vector normalized to unit energy (equation 44).
fn normalize(y: &[i32; SCALE_FACTORS]) -> [f32; SCALE_FACTORS] {
    let energy: i32 = y.iter().map(|v| v * v).sum();
    if energy == 0 {
        return [0.0; SCALE_FACTORS];
    }
    let g = 1.0 / libm::sqrtf(energy as f32);
    y.map(|v| v as f32 * g)
}

/// Shape and gain of minimum distortion with the target (equations 55 and 56).
fn select(t2rot: &[f32; SCALE_FACTORS], xq: &[[f32; SCALE_FACTORS]; 4]) -> (Shape, usize) {
    let mut best = (f32::INFINITY, Shape::Regular, 0);
    for (shape, xqj) in Shape::ALL.into_iter().zip(xq) {
        for (gain, &g) in shape.gains().iter().enumerate() {
            let distortion: f32 = t2rot
                .iter()
                .zip(xqj)
                .map(|(&t, &x)| {
                    let e = t - g * x;
                    e * e
                })
                .sum();
            if distortion < best.0 {
                best = (distortion, shape, gain);
            }
        }
    }
    (best.1, best.2)
}

/// MPVQ enumeration of a pulse vector (`MPVQenum`, section 3.3.7.3.3.8),
/// returns the shape index and the leading sign index.
fn mpvq_enumerate(vec: &[i32]) -> (u32, u32) {
    const NO_SIGN: u32 = 0x8000_0000;
    let mut next_sign = NO_SIGN;
    let mut pulses = 0;
    let mut index = 0;
    let mut row = 0;
    let mut offset = MPVQ_OFFSETS[0][0];
    for pos in (0..vec.len()).rev() {
        let val = vec[pos];
        if next_sign & NO_SIGN == 0 && val != 0 {
            index = 2 * index + next_sign;
        }
        if val < 0 {
            next_sign = 1;
        }
        if val > 0 {
            next_sign = 0;
        }
        index += offset;
        pulses += val.unsigned_abs() as usize;
        if pos != 0 {
            row += 1;
        }
        offset = MPVQ_OFFSETS[row][pulses];
    }
    (index, next_sign)
}

/// MPVQ de-enumeration of the shape index `index` and leading sign index
/// `leading_sign` into the `pulses` pulses of `vec` (`MPVQdeenum`, section
/// 3.4.7.2.2.1).
fn mpvq_deenumerate(pulses: i32, leading_sign: u32, mut index: u32, vec: &mut [i32]) {
    vec.fill(0);
    let mut sign = if leading_sign != 0 { -1 } else { 1 };
    let mut remaining = pulses;
    let dimensions = vec.len();
    for (pos, v) in vec.iter_mut().enumerate() {
        if index == 0 {
            *v = sign * remaining;
            break;
        }
        // Offsets of the dimension left, from this position to the end.
        let offsets = &MPVQ_OFFSETS[dimensions - 1 - pos];
        let mut k = remaining;
        while index < offsets[k as usize] {
            k -= 1;
        }
        index -= offsets[k as usize];
        let delta = remaining - k;
        if delta != 0 {
            *v = sign * delta;
            sign = if index & 1 != 0 { -1 } else { 1 };
            index >>= 1;
            remaining -= delta;
        }
    }
}

/// Quantized scale factors `scfQ` (equation 62).
fn synthesize(
    ind_lf: usize,
    ind_hf: usize,
    gain: f32,
    xq: &[f32; SCALE_FACTORS],
) -> [f32; SCALE_FACTORS] {
    let st1 = stage1_vector(ind_lf, ind_hf);
    let mut scfq = [0.0; SCALE_FACTORS];
    for (n, q) in scfq.iter_mut().enumerate() {
        let rotated: f32 = (0..SCALE_FACTORS).map(|col| xq[col] * D[n][col]).sum();
        *q = st1[n] + gain * rotated;
    }
    scfq
}

/// Vector quantization of the scale factors (section 3.3.7.3), returns the
/// indices and the quantized scale factors.
pub(crate) fn quantize(scf: &[f32; SCALE_FACTORS]) -> (Indices, [f32; SCALE_FACTORS]) {
    let ind_lf = nearest(&LFCB, &scf[..SPLIT]);
    let ind_hf = nearest(&HFCB, &scf[SPLIT..]);
    let st1 = stage1_vector(ind_lf, ind_hf);
    let mut r1 = [0.0; SCALE_FACTORS];
    for n in 0..SCALE_FACTORS {
        r1[n] = scf[n] - st1[n];
    }

    let t2rot = rotate(&r1);
    let pulses = search_shapes(&t2rot);
    let xq = pulses.map(|y| normalize(&y));
    let (shape, gain) = select(&t2rot, &xq);

    // Enumeration of the selected shape (table 3.12)
    let y = &pulses[shape as usize];
    let ((idx_a, ls_a), (idx_b, ls_b)) = match shape {
        Shape::Regular => (mpvq_enumerate(&y[..SET_A]), mpvq_enumerate(&y[SET_A..])),
        Shape::RegularLf => (mpvq_enumerate(&y[..SET_A]), (0, 0)),
        Shape::OutlierNear | Shape::OutlierFar => (mpvq_enumerate(y), (0, 0)),
    };
    let indices = Indices {
        ind_lf,
        ind_hf,
        shape,
        gain,
        ls_a,
        idx_a,
        ls_b,
        idx_b,
    };
    let scfq = synthesize(ind_lf, ind_hf, shape.gains()[gain], &xq[shape as usize]);
    (indices, scfq)
}

/// Quantized scale factors of decoded indices (section 3.4.7.2).
pub(crate) fn dequantize(indices: &Indices) -> [f32; SCALE_FACTORS] {
    let mut y = [0i32; SCALE_FACTORS];
    match indices.shape {
        Shape::Regular => {
            mpvq_deenumerate(REGULAR_PULSES, indices.ls_a, indices.idx_a, &mut y[..SET_A]);
            mpvq_deenumerate(1, indices.ls_b, indices.idx_b, &mut y[SET_A..]);
        }
        Shape::RegularLf => {
            mpvq_deenumerate(REGULAR_PULSES, indices.ls_a, indices.idx_a, &mut y[..SET_A]);
        }
        Shape::OutlierNear => {
            mpvq_deenumerate(OUTLIER_NEAR_PULSES, indices.ls_a, indices.idx_a, &mut y)
        }
        Shape::OutlierFar => {
            mpvq_deenumerate(OUTLIER_FAR_PULSES, indices.ls_a, indices.idx_a, &mut y)
        }
    }
    let gains = indices.shape.gains();
    let gain = gains[indices.gain.min(gains.len() - 1)];
    synthesize(indices.ind_lf, indices.ind_hf, gain, &normalize(&y))
}

/// Writes the quantization indices to the side information (section 3.3.13.3).
pub(crate) fn write(w: &mut Writer, indices: &Indices) {
    let shape = indices.shape;
    w.write_uint(indices.ind_lf as u32, STAGE1_BITS);
    w.write_uint(indices.ind_hf as u32, STAGE1_BITS);
    w.write_bit(shape.submode_msb());
    w.write_uint(
        (indices.gain >> shape.gain_lsb_bits()) as u32,
        shape.gain_msb_bits(),
    );
    w.write_bit(indices.ls_a);

    // Joint index (equations 58 to 61)
    let gain_lsb = (indices.gain & 1) as u32;
    let joint = match shape {
        Shape::Regular => (2 * indices.idx_b + indices.ls_b + 2) * SIZE_REGULAR_A + indices.idx_a,
        Shape::RegularLf => gain_lsb * SIZE_REGULAR_A + indices.idx_a,
        Shape::OutlierNear => indices.idx_a,
        Shape::OutlierFar => SIZE_OUTLIER_NEAR + gain_lsb + 2 * indices.idx_a,
    };
    let low_bits = if shape.submode_msb() == 0 {
        REGULAR_JOINT_LOW_BITS
    } else {
        OUTLIER_JOINT_LOW_BITS
    };
    w.write_uint(joint, low_bits);
    w.write_uint(joint >> low_bits, JOINT_HIGH_BITS);
}

/// Reads the quantization indices from the side information (section
/// 3.4.7.2), `None` on a bit error.
pub(crate) fn read(r: &mut Reader) -> Option<Indices> {
    let ind_lf = r.read_uint(STAGE1_BITS) as usize;
    let ind_hf = r.read_uint(STAGE1_BITS) as usize;
    let submode_msb = r.read_bit();
    let gain_msb_bits = if submode_msb == 0 { 1 } else { 2 };
    let mut gain = r.read_uint(gain_msb_bits) as usize;
    let ls_a = r.read_bit();
    let mut indices = Indices {
        ind_lf,
        ind_hf,
        ls_a,
        ..Indices::default()
    };

    let submode_lsb;
    if submode_msb == 0 {
        let joint = r.read_uint(REGULAR_JOINT_LOW_BITS)
            | (r.read_uint(JOINT_HIGH_BITS) << REGULAR_JOINT_LOW_BITS);
        if joint >= REGULAR_JOINT_SLOTS * SIZE_REGULAR_A {
            return None;
        }
        let slot = joint / SIZE_REGULAR_A;
        indices.idx_a = joint - slot * SIZE_REGULAR_A;
        if slot < 2 {
            submode_lsb = 1;
            gain = (gain << 1) + slot as usize;
        } else {
            submode_lsb = 0;
            indices.idx_b = (slot - 2) >> 1;
            indices.ls_b = (slot - 2) & 1;
        }
    } else {
        let joint = r.read_uint(OUTLIER_JOINT_LOW_BITS)
            | (r.read_uint(JOINT_HIGH_BITS) << OUTLIER_JOINT_LOW_BITS);
        if joint >= SIZE_OUTLIER_NEAR + 2 * SIZE_OUTLIER_FAR {
            return None;
        }
        if joint >= SIZE_OUTLIER_NEAR {
            let far = joint - SIZE_OUTLIER_NEAR;
            submode_lsb = 1;
            gain = (gain << 1) + (far & 1) as usize;
            indices.idx_a = far >> 1;
        } else {
            submode_lsb = 0;
            indices.idx_a = joint;
        }
    }
    indices.shape = Shape::from_submodes(submode_msb, submode_lsb);
    indices.gain = gain;
    Some(indices)
}

#[cfg(test)]
mod tests {
    use super::{
        Indices, SCALE_FACTORS, Shape, dequantize, mpvq_deenumerate, mpvq_enumerate, quantize,
        rotate, search_shapes, stage1_vector,
    };
    use crate::bitstream::{Reader, Writer};
    use crate::sns::SIDE_INFO_BITS;
    use crate::test_vectors::{FRAME_1_SCF, FRAME_1_SCFQ, SHAPE_3_SCF, SHAPE_3_SCFQ};

    /// Asserts that two vectors are equal within `tolerance`.
    fn assert_close(got: &[f32], want: &[f64], tolerance: f64) {
        for (i, (&g, &w)) in got.iter().zip(want).enumerate() {
            assert!(
                (f64::from(g) - w).abs() <= tolerance,
                "[{i}]: got {g}, want {w}"
            );
        }
    }

    #[test]
    fn mpvq_round_trip() {
        let vectors: [&[i32]; 5] = [
            &[-10, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            &[0, 0, 0, 0, 0, 0, 0, 0, 0, 10],
            &[3, -2, 0, 1, 0, 0, -1, 2, 0, 1],
            &[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -7],
            &[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, -1, 1, -1, 1],
        ];
        for v in vectors {
            let pulses = v.iter().map(|x| x.abs()).sum();
            let (index, leading_sign) = mpvq_enumerate(v);
            let mut out = vec![0; v.len()];
            mpvq_deenumerate(pulses, leading_sign, index, &mut out);
            assert_eq!(out, v);
        }
    }

    /// First frame of appendix C.3.2 (16 kHz, 10 ms).
    #[test]
    fn quantization_of_appendix_c_frame() {
        let scf = FRAME_1_SCF.map(|v| v as f32);
        let r1: [f32; SCALE_FACTORS] = core::array::from_fn(|n| scf[n] - stage1_vector(25, 8)[n]);
        let pulses = search_shapes(&rotate(&r1));
        assert_eq!(
            pulses[0],
            [-1, 1, -2, -1, 0, 3, 0, -2, 0, 0, 0, -1, 0, 0, 0, 0]
        );
        assert_eq!(
            pulses[2],
            [-1, 0, -1, -1, 0, 2, 0, -1, 0, 0, 0, -1, -1, 0, 0, 0]
        );
        assert_eq!(
            pulses[3],
            [0, 0, -1, -1, 0, 2, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0]
        );

        let (indices, scfq) = quantize(&scf);
        let expected = Indices {
            ind_lf: 25,
            ind_hf: 8,
            shape: Shape::Regular,
            gain: 0,
            ls_a: 1,
            idx_a: 865_837,
            ls_b: indices.ls_b,
            idx_b: 1,
        };
        assert_eq!(indices, expected);
        assert_close(&scfq, &FRAME_1_SCFQ, 1e-4);
    }

    /// Far outlier shape vectors of appendix C.3.3.
    #[test]
    fn quantization_with_the_far_outlier_shape() {
        let (indices, scfq) = quantize(&SHAPE_3_SCF.map(|v| v as f32));
        assert_eq!((indices.ind_lf, indices.ind_hf), (4, 27));
        assert_eq!(indices.shape, Shape::OutlierFar);
        assert_eq!((indices.gain, indices.ls_a, indices.idx_a), (3, 1, 61_886));
        assert_close(&scfq, &SHAPE_3_SCFQ, 1e-4);
    }

    #[test]
    fn indices_round_trip_through_the_bitstream() {
        for scf in [FRAME_1_SCF, SHAPE_3_SCF] {
            let (indices, scfq) = quantize(&scf.map(|v| v as f32));
            let mut frame = [0u8; 20];
            let mut w = Writer::new(&mut frame);
            super::write(&mut w, &indices);
            assert_eq!(w.nbits_side(), SIDE_INFO_BITS as i32);
            w.finish();
            let decoded = super::read(&mut Reader::new(&frame)).unwrap();
            assert_eq!(decoded.shape, indices.shape);
            assert_eq!(dequantize(&decoded), scfq);
        }
    }
}
