pub mod car;
pub mod statics;
pub mod water;
pub mod gameplay;
pub mod bodies;
pub mod session;
pub mod paths;
pub use session::{open, open_file, OpenOptions};

fn sort_by_key<T, K: Ord>(values: &mut Vec<T>, key: impl Fn(&T) -> K) {
    let mut entries: Vec<_> = std::mem::take(values).into_iter().enumerate()
        .map(|(i, value)| (key(&value), i, value)).collect();
    gearbox_engine::numeric::quicksort(&mut entries, |a, b| match a.0.cmp(&b.0).then(a.1.cmp(&b.1)) {
        std::cmp::Ordering::Less => -1, std::cmp::Ordering::Equal => 0, std::cmp::Ordering::Greater => 1,
    });
    values.extend(entries.into_iter().map(|(_, _, value)| value));
}
