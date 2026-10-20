use anyhow::{Result, bail};
use serde::Serialize;

pub const TICK_MS: i32 = 10;

#[derive(Debug, Clone, Serialize)]
pub struct PlayerInputs {
    pub version: i32,
    pub start_offset_ms: Option<i32>,
    pub ticks: usize,
    #[serde(skip)]
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize)]
pub struct TickInput {
    pub steer: i8,
    pub gas: bool,
    pub brake: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct Event {
    pub tick: usize,
    pub kind: EventKind,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq)]
pub enum EventKind {
    Respawn,
    SecondaryRespawn,
    Horn(bool),
}

#[derive(Debug, Clone, Serialize)]
pub struct DecodedInputs {
    pub start_offset_ms: i32,
    pub ticks: Vec<TickInput>,
    pub events: Vec<Event>,
    pub truncated_at: Option<usize>,
    pub state_words: Vec<(usize, u64)>,
    pub extra_words: Vec<(usize, u32)>,
}

struct Bits<'a> {
    data: &'a [u8],
    pos: usize,
}

struct OutOfBits;

impl<'a> Bits<'a> {
    fn bit(&mut self) -> Result<bool, OutOfBits> {
        let byte = *self.data.get(self.pos / 8).ok_or(OutOfBits)?;
        let v = byte & (1 << (self.pos % 8)) != 0;
        self.pos += 1;
        Ok(v)
    }

    fn num(&mut self, bits: u32) -> Result<u64, OutOfBits> {
        let mut v = 0u64;
        for i in 0..bits {
            v |= (self.bit()? as u64) << i;
        }
        Ok(v)
    }

    fn rest_is_zero(&self) -> bool {
        let total = self.data.len() * 8;
        (self.pos..total).all(|p| self.data[p / 8] & (1 << (p % 8)) == 0)
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    NotStarted,
    Character,
    Vehicle,
}

impl PlayerInputs {
    pub fn decode(&self) -> Result<DecodedInputs> {
        if self.version <= 8 {
            bail!("input version {} is ShootMania-style and not supported", self.version);
        }
        let state_bits = if self.version == 11 { 33 } else { 34 };
        let mut r = Bits { data: &self.data, pos: 0 };
        let mut mode = Mode::NotStarted;
        let mut cur = TickInput::default();
        let mut horn = false;
        let mut state = 0;
        let mut out = Vec::with_capacity(self.ticks);
        let mut events = Vec::new();
        let mut truncated_at = None;
        let mut state_words = Vec::new();
        let mut extra_words = Vec::new();

        for tick in 0..self.ticks {
            let step = (|| -> Result<Result<(), String>, OutOfBits> {
                if !r.bit()? {
                    if r.bit()? {
                        let states = r.num(2)?;
                        state = (state & !(3 << 5)) | states << 5;
                        state_words.push((tick, state));
                        if mode == Mode::Vehicle {
                            let h = states & 2 != 0;
                            if h != horn {
                                horn = h;
                                events.push(Event { tick, kind: EventKind::Horn(h) });
                            }
                        }
                    } else {
                        let states = r.num(state_bits)?;
                        state = states;
                        state_words.push((tick, state));
                        mode = match states & 0xF {
                            0 => Mode::NotStarted,
                            2..=4 => Mode::Vehicle,
                            k @ (0xC | 0xD) => {
                                return Ok(Err(format!(
                                    "input kind 0x{k:X} (float steering) at tick {tick} is not decoded yet"
                                )));
                            }
                            _ => Mode::Character,
                        };
                        if mode == Mode::Vehicle {
                            let h = states & (1 << 6) != 0;
                            if h != horn {
                                horn = h;
                                events.push(Event { tick, kind: EventKind::Horn(h) });
                            }
                        }
                        if states >> 31 & 1 != 0 {
                            events.push(Event { tick, kind: EventKind::Respawn });
                        }
                        if states >> 33 & 1 != 0 {
                            events.push(Event { tick, kind: EventKind::SecondaryRespawn });
                        }
                    }
                }
                if !r.bit()? {
                    let w = r.num(32)? as u32;
                    extra_words.push((tick, w));
                }
                match mode {
                    Mode::Vehicle => {
                        if !r.bit()? {
                            cur.steer = r.num(8)? as u8 as i8;
                            cur.gas = r.bit()?;
                            cur.brake = r.bit()?;
                        }
                    }
                    Mode::Character => {
                        if !r.bit()? {
                            r.num(8)?;
                        }
                    }
                    Mode::NotStarted => {}
                }
                Ok(Ok(()))
            })();
            match step {
                Ok(Ok(())) => out.push(cur),
                Ok(Err(msg)) => bail!(msg),
                Err(OutOfBits) => {
                    truncated_at = Some(tick);
                    out.resize(self.ticks, cur);
                    break;
                }
            }
        }
        if truncated_at.is_none() && !r.rest_is_zero() {
            bail!("input bitstream has {} unread non-zero bits", self.data.len() * 8 - r.pos);
        }
        Ok(DecodedInputs {
            start_offset_ms: self.start_offset_ms.unwrap_or(0),
            ticks: out,
            events,
            truncated_at,
            state_words,
            extra_words,
        })
    }
}
