use super::mpc_constants::*;
use super::ProcessedState;

/// MPC制御器 (ADMM QP ソルバー)
/// condensed form の box制約付きQPを固定反復ADMMで解く
pub struct MpcController {
    u: [f32; MPC_N],
    z: [f32; MPC_N],
    w: [f32; MPC_N],
    force: f32,
    first_call: bool,
}

impl MpcController {
    pub fn new() -> Self {
        Self {
            u: [0.0; MPC_N],
            z: [0.0; MPC_N],
            w: [0.0; MPC_N],
            force: 0.0,
            first_call: true,
        }
    }

    /// MPC制御力を計算（100Hzで呼ばれる）
    pub fn compute_force(&mut self, state: &ProcessedState) -> f32 {
        let x = [state.position, state.velocity, state.theta, state.theta_dot];

        // Warm-start: 前回の解を1ステップシフト
        if !self.first_call {
            for i in 0..MPC_N - 1 {
                self.u[i] = self.u[i + 1];
                self.z[i] = self.z[i + 1];
            }
            self.u[MPC_N - 1] = self.u[MPC_N - 2];
            self.z[MPC_N - 1] = self.z[MPC_N - 2];
            self.w = [0.0; MPC_N];
        } else {
            self.first_call = false;
        }

        // f = F * x_current (勾配ベクトル、オンライン計算)
        let mut f = [0.0f32; MPC_N];
        for i in 0..MPC_N {
            let row_offset = i * MPC_NX;
            let mut sum = 0.0f32;
            for j in 0..MPC_NX {
                sum += MPC_F[row_offset + j] * x[j];
            }
            f[i] = sum;
        }

        // ADMM反復
        for _ in 0..MPC_MAX_ITER {
            // U更新: U = H_inv_rho * (-f + rho*(Z - W))
            let mut rhs = [0.0f32; MPC_N];
            for i in 0..MPC_N {
                rhs[i] = -f[i] + MPC_RHO * (self.z[i] - self.w[i]);
            }

            for i in 0..MPC_N {
                let row_offset = i * MPC_N;
                let mut sum = 0.0f32;
                for j in 0..MPC_N {
                    sum += MPC_H_INV_RHO[row_offset + j] * rhs[j];
                }
                self.u[i] = sum;
            }

            // Z更新: box制約への射影 (clamp)
            for i in 0..MPC_N {
                let val = self.u[i] + self.w[i];
                self.z[i] = if val < MPC_U_MIN {
                    MPC_U_MIN
                } else if val > MPC_U_MAX {
                    MPC_U_MAX
                } else {
                    val
                };
            }

            // W更新: 双対変数
            for i in 0..MPC_N {
                self.w[i] += self.u[i] - self.z[i];
            }
        }

        self.force = self.u[0];
        self.force
    }

    /// 保持中の力指令を返す（MPC非更新tick用）
    pub fn held_force(&self) -> f32 {
        self.force
    }

    pub fn reset(&mut self) {
        self.u = [0.0; MPC_N];
        self.z = [0.0; MPC_N];
        self.w = [0.0; MPC_N];
        self.force = 0.0;
        self.first_call = true;
    }
}
