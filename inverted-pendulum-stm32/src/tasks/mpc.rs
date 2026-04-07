use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::controller::mpc::MpcController;
use crate::controller::ProcessedState;

/// バランスループ → MPCタスク: 前処理済み状態 (50Hzでシグナル)
pub static MPC_STATE_SIGNAL: Signal<CriticalSectionRawMutex, ProcessedState> = Signal::new();

/// MPCタスク → バランスループ: 計算済み力 [N] (f32のビット表現)
static MPC_FORCE: AtomicU32 = AtomicU32::new(0);

/// リセット要求フラグ
static MPC_NEEDS_RESET: AtomicBool = AtomicBool::new(false);

/// MPCの最新力指令を取得
pub fn get_mpc_force() -> f32 {
    f32::from_bits(MPC_FORCE.load(Ordering::Relaxed))
}

/// MPCコントローラのリセットを要求
pub fn request_mpc_reset() {
    MPC_NEEDS_RESET.store(true, Ordering::Relaxed);
    MPC_FORCE.store(0f32.to_bits(), Ordering::Relaxed);
}

/// MPCタスク: Signal待ちで非同期実行、制御ループをブロックしない
#[embassy_executor::task]
pub async fn mpc_task() {
    let mut controller = MpcController::new();

    loop {
        let state = MPC_STATE_SIGNAL.wait().await;

        if MPC_NEEDS_RESET.swap(false, Ordering::Relaxed) {
            controller.reset();
        }

        let force = controller.compute_force(&state);
        MPC_FORCE.store(force.to_bits(), Ordering::Relaxed);
    }
}
