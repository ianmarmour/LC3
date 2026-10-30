//! Numerical helpers shared by the codec modules.

/// Rounds to the nearest integer, ties away from zero (`nint` of section 3.1.3).
pub(crate) fn nint(x: f32) -> i32 {
    libm::roundf(x) as i32
}

#[cfg(test)]
mod tests {
    use super::nint;

    #[test]
    fn rounds_ties_away_from_zero() {
        assert_eq!(nint(-4.5), -5);
        assert_eq!(nint(-3.2), -3);
        assert_eq!(nint(3.2), 3);
        assert_eq!(nint(4.5), 5);
    }
}
