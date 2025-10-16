// Statistical analysis (stub)
pub fn min_max<T: Ord + Copy>(data: &[T]) -> Option<(T, T)> {
    let mut it = data.iter().copied();
    let first = it.next()?;
    let mut min = first;
    let mut max = first;
    for v in it {
        if v < min { min = v; }
        if v > max { max = v; }
    }
    Some((min, max))
}

