use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
extern crate alloc;
use alloc::string::String;
use esp_radio::wifi::{
    AccessPointConfig, AuthMethod, ClientConfig, ModeConfig, WifiController, WifiEvent,
};
use inverted_pendulum_protocol::SensorData;
use log::{info, warn};

use crate::led::{WifiStatus, WIFI_STATUS};

/// WiFi経由でセンサデータを受け取るチャネル（将来のWebSocket用）
pub static WIFI_SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorData, 2> = Channel::new();

/// STA接続リトライ回数
const MAX_STA_RETRIES: u8 = 3;

/// WiFi SSID (コンパイル時環境変数またはデフォルト)
const WIFI_SSID: &str = match option_env!("WIFI_SSID") {
    Some(s) => s,
    None => "InvertedPendulum_Network",
};

/// WiFi パスワード (コンパイル時環境変数またはデフォルト)
const WIFI_PASSWORD: &str = match option_env!("WIFI_PASSWORD") {
    Some(s) => s,
    None => "pendulum123",
};

/// AP モード SSID
const AP_SSID: &str = "InvertedPendulum";

/// WiFi接続管理タスク: STA接続 → 失敗時APモードへフォールバック
#[embassy_executor::task]
pub async fn wifi_task(mut controller: WifiController<'static>) {
    info!("[wifi] task started");
    info!("[wifi] STA SSID: {}", WIFI_SSID);

    // STA モードで接続試行
    WIFI_STATUS.signal(WifiStatus::Connecting);

    let sta_config = ModeConfig::Client(
        ClientConfig::default()
            .with_ssid(String::from(WIFI_SSID))
            .with_password(String::from(WIFI_PASSWORD))
            .with_auth_method(if WIFI_PASSWORD.is_empty() {
                AuthMethod::None
            } else {
                AuthMethod::Wpa2Personal
            }),
    );

    if let Err(e) = controller.set_config(&sta_config) {
        warn!("[wifi] set STA config failed: {:?}", e);
        start_ap_mode(&mut controller).await;
        return;
    }

    if let Err(e) = controller.start_async().await {
        warn!("[wifi] start failed: {:?}", e);
        start_ap_mode(&mut controller).await;
        return;
    }

    // STA接続リトライ
    let mut connected = false;
    for attempt in 1..=MAX_STA_RETRIES {
        info!("[wifi] STA connect attempt {}/{}", attempt, MAX_STA_RETRIES);
        match controller.connect_async().await {
            Ok(()) => {
                info!("[wifi] STA connected");
                WIFI_STATUS.signal(WifiStatus::StaConnected);
                connected = true;
                break;
            }
            Err(e) => {
                warn!("[wifi] STA connect failed: {:?}", e);
                if attempt < MAX_STA_RETRIES {
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        }
    }

    if !connected {
        warn!("[wifi] STA connection failed after {} retries, switching to AP mode", MAX_STA_RETRIES);
        // STAモード停止してAPモードへ
        let _ = controller.stop_async().await;
        start_ap_mode(&mut controller).await;
        return;
    }

    // STA接続維持ループ: 切断検知 → 再接続
    loop {
        controller
            .wait_for_event(WifiEvent::StaDisconnected)
            .await;
        warn!("[wifi] STA disconnected, reconnecting...");
        WIFI_STATUS.signal(WifiStatus::Connecting);

        let mut reconnected = false;
        for attempt in 1..=MAX_STA_RETRIES {
            info!("[wifi] reconnect attempt {}/{}", attempt, MAX_STA_RETRIES);
            match controller.connect_async().await {
                Ok(()) => {
                    info!("[wifi] STA reconnected");
                    WIFI_STATUS.signal(WifiStatus::StaConnected);
                    reconnected = true;
                    break;
                }
                Err(e) => {
                    warn!("[wifi] reconnect failed: {:?}", e);
                    if attempt < MAX_STA_RETRIES {
                        Timer::after(Duration::from_secs(1)).await;
                    }
                }
            }
        }

        if !reconnected {
            warn!("[wifi] reconnection failed, switching to AP mode");
            let _ = controller.stop_async().await;
            start_ap_mode(&mut controller).await;
            return;
        }
    }
}

/// APモードを開始する
async fn start_ap_mode(controller: &mut WifiController<'static>) {
    info!("[wifi] starting AP mode with SSID: {}", AP_SSID);
    WIFI_STATUS.signal(WifiStatus::ApMode);

    let ap_config = ModeConfig::AccessPoint(
        AccessPointConfig::default()
            .with_ssid(String::from(AP_SSID))
            .with_auth_method(AuthMethod::None),
    );

    if let Err(e) = controller.set_config(&ap_config) {
        warn!("[wifi] set AP config failed: {:?}", e);
        WIFI_STATUS.signal(WifiStatus::Disconnected);
        return;
    }

    if let Err(e) = controller.start_async().await {
        warn!("[wifi] AP start failed: {:?}", e);
        WIFI_STATUS.signal(WifiStatus::Disconnected);
        return;
    }

    info!("[wifi] AP mode active");

    // APモードは維持し続ける
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
