use embassy_futures::select;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;

#[derive(Debug, Clone, Copy)]
pub struct QeiState {
    pub pulses: i32,
    pub revolutions: i32,
    curr_state: u8,
    prev_state: u8,
}

pub struct Qei<'a> {
    chan_a: ExtiInput<'a, Async>,
    chan_b: ExtiInput<'a, Async>,
    pulses_per_rev: i32,
    state: QeiState,
}

impl<'a> Qei<'a> {
    pub fn new(chan_a: ExtiInput<'a, Async>, chan_b: ExtiInput<'a, Async>, pulses_per_rev: i32) -> Self {
        let a = if chan_a.is_high() { 1u8 } else { 0 };
        let b = if chan_b.is_high() { 1u8 } else { 0 };
        let init = (a << 1) | b;

        Self {
            chan_a,
            chan_b,
            pulses_per_rev,
            state: QeiState {
                pulses: 0,
                revolutions: 0,
                curr_state: init,
                prev_state: init,
            },
        }
    }

    /// Wait for an edge and decode one X4 transition. Returns updated state.
    pub async fn step(&mut self) -> QeiState {
        select::select(
            self.chan_a.wait_for_any_edge(),
            self.chan_b.wait_for_any_edge(),
        )
        .await;

        self.decode();
        self.state
    }

    pub fn reset(&mut self) {
        self.state.pulses = 0;
        self.state.revolutions = 0;
    }

    fn decode(&mut self) {
        let a = if self.chan_a.is_high() { 1u8 } else { 0 };
        let b = if self.chan_b.is_high() { 1u8 } else { 0 };
        let curr = (a << 1) | b;
        let prev = self.state.curr_state;

        self.state.curr_state = curr;

        // X4 decoding
        if ((curr ^ prev) != 0x3) && (curr != prev) {
            let change = (prev & 0x1) ^ ((curr & 0x2) >> 1);
            if change == 0 {
                self.state.pulses += 1;
            } else {
                self.state.pulses -= 1;
            }
        }

        // Revolution counting
        if self.pulses_per_rev > 0 {
            if self.state.pulses >= self.pulses_per_rev {
                self.state.revolutions += 1;
                self.state.pulses -= self.pulses_per_rev;
            } else if self.state.pulses <= -self.pulses_per_rev {
                self.state.revolutions -= 1;
                self.state.pulses += self.pulses_per_rev;
            }
        }

        self.state.prev_state = curr;
    }
}
