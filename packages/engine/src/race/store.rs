use super::{NO_RUN, RaceRun, Racer};

pub const ENTRIES_PER_CHUNK: usize = 32;
const RECORD_ID_NAMESPACE: u32 = 0x0100_0000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry { pub time: u32, pub waypoint: u16 }
impl Default for Entry {
    fn default() -> Self { Self { time: NO_RUN, waypoint: u16::MAX } }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Chunk {
    pub id: u32,
    pub run: u32,
    pub sequence: u32,
    pub count: u32,
    pub entries: [Entry; ENTRIES_PER_CHUNK],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IdSequence {
    pub owner: u8,
    pub first: u32,
    pub second: u32,
    pub first_limit: u16,
    pub second_limit: u16,
    pub used: u16,
}
impl IdSequence {
    pub fn take(&mut self) -> u32 {
        let id = self.first.min(self.second).wrapping_add(self.used as u32) | RECORD_ID_NAMESPACE;
        self.used = self.used.wrapping_add(1);
        let limit = if self.first < self.second { self.first_limit } else { self.second_limit };
        if self.used >= limit { todo!("refill a race id block"); }
        id
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Store {
    pub runs: Vec<RaceRun>,
    pub chunks: Vec<Chunk>,
    pub ids: Vec<IdSequence>,
}
impl Store {
    pub fn run(&self, id: u32) -> Option<&RaceRun> {
        if id == NO_RUN { None } else { self.runs.iter().find(|run| run.id == id) }
    }

    pub fn chunk_indices(&self, run: u32) -> Vec<usize> {
        let mut indices: Vec<_> = if run == NO_RUN { Vec::new() } else {
            self.chunks.iter().enumerate().filter(|(_, chunk)| chunk.run == run)
                .map(|(index, _)| index).collect()
        };
        crate::numeric::quicksort(&mut indices, |&left, &right| {
            let order = self.chunks[left].sequence.cmp(&self.chunks[right].sequence)
                .then_with(|| left.cmp(&right));
            match order {
                std::cmp::Ordering::Less => -1,
                std::cmp::Ordering::Equal => 0,
                std::cmp::Ordering::Greater => 1,
            }
        });
        indices
    }

    pub fn entries(&self, run: u32) -> impl Iterator<Item = &Entry> {
        self.chunk_indices(run).into_iter().flat_map(|index| {
            self.chunks[index].entries.iter().take_while(|entry| entry.time != NO_RUN)
        })
    }

    pub fn entry_count(&self, run: u32) -> u32 {
        let indices = self.chunk_indices(run);
        let Some(&last) = indices.last() else { return 0; };
        let last_count = self.chunks[last].entries.iter()
            .take_while(|entry| entry.time != NO_RUN).count() as u32;
        (indices.len() as u32 - 1).wrapping_mul(ENTRIES_PER_CHUNK as u32).wrapping_add(last_count)
    }

    fn next_id(&mut self, owner: u8) -> u32 {
        self.ids.iter_mut().find(|block| block.owner == owner)
            .expect("player has a race id block").take()
    }

    pub fn record(&mut self, owner: u8, runs: [u32; 2], entry: Entry, finish: bool) {
        if finish {
            for key in runs {
                if key == NO_RUN { continue; }
                if let Some(run) = self.runs.iter_mut().find(|run| run.id == key) { run.finish = entry.time; }
            }
        }
        let lasts = runs.map(|key| self.chunk_indices(key).last().copied());
        let destinations = [0, 1].map(|index| self.destination(owner, runs[index], lasts[index]));
        for (chunk, slot) in destinations {
            let chunk = &mut self.chunks[chunk];
            chunk.entries[slot] = entry;
            chunk.count = chunk.count.wrapping_add(1);
        }
    }

    fn destination(&mut self, owner: u8, run: u32, last: Option<usize>) -> (usize, usize) {
        if let Some(index) = last {
            if let Some(slot) = self.chunks[index].entries.iter().position(|entry| entry.time == NO_RUN) {
                return (index, slot);
            }
        }
        let sequence = last.map_or(0, |index| self.chunks[index].sequence.wrapping_add(1));
        let id = self.next_id(owner);
        let index = self.chunks.len();
        self.chunks.push(Chunk {
            id, run, sequence, count: 0, entries: [Entry::default(); ENTRIES_PER_CHUNK],
        });
        (index, 0)
    }
}

impl Racer {
    pub fn close_lap(&mut self, store: &Store) {
        let index = if self.player.lap_run == NO_RUN { None } else {
            store.runs.iter().position(|run| run.id == self.player.lap_run)
        };
        self.closed_lap = index;
        self.player.lap_run = NO_RUN;
        self.open_lap = None;
    }

    pub fn open_lap(&mut self, store: &mut Store, time: u32) {
        let id = store.next_id(self.player.tag);
        self.open_lap = Some(store.runs.len());
        store.runs.push(RaceRun { id, finish: NO_RUN, start: time });
        self.player.lap_run = id;
    }
}
