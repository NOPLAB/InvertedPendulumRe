use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use inverted_pendulum_protocol::{CommandType, SensorData, SENSOR_DATA_RAW_SIZE};
use log::{info, warn};
use trouble_host::prelude::*;

use crate::led::{BleStatus, BLE_STATUS};
use crate::uart::COMMAND_CHANNEL;

/// BLE経由でセンサデータを受け取るチャネル
pub static BLE_SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorData, 2> = Channel::new();

/// 最大接続数
const CONNECTIONS_MAX: usize = 1;
/// L2CAPチャネル数 (Signal + ATT)
const L2CAP_CHANNELS_MAX: usize = 2;

// GATT Server定義
#[gatt_server]
struct Server {
    pendulum_service: PendulumService,
}

// 倒立振子サービス
#[gatt_service(uuid = "ff94ca6f-91a5-4048-b805-755bca71075e")]
struct PendulumService {
    /// TX: センサデータ通知 (26バイト)
    #[characteristic(uuid = "b2322be9-f43c-4431-8ec1-d90e3b8bd848", read, notify)]
    sensor_data: [u8; SENSOR_DATA_RAW_SIZE],
    /// RX: コマンド受信 (3バイト)
    #[characteristic(uuid = "a3308588-5d66-4643-b0ef-0df52539d950", write)]
    command: [u8; 3],
}

/// BLEタスク: GATT Server起動、アドバタイズ、接続管理
#[embassy_executor::task]
pub async fn ble_task(controller: ExternalController<esp_radio::ble::controller::BleConnector<'static>, 20>) {
    let address: Address = Address::random([0xff, 0x9a, 0x1b, 0x06, 0xe5, 0xff]);
    info!("[ble] address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    info!("[ble] starting GATT server");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "InvertedPendulum",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    let _ = join(ble_runner(runner), async {
        loop {
            BLE_STATUS.signal(BleStatus::Advertising);
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    BLE_STATUS.signal(BleStatus::Connected);
                    let a = gatt_events_task(&server, &conn);
                    let b = notify_task(&server, &conn);
                    select(a, b).await;
                    BLE_STATUS.signal(BleStatus::Disconnected);
                }
                Err(e) => {
                    warn!("[ble] advertise error: {:?}", e);
                    BLE_STATUS.signal(BleStatus::Disconnected);
                }
            }
        }
    })
    .await;
}

/// BLEスタックランナー
async fn ble_runner<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            warn!("[ble] runner error: {:?}", e);
        }
    }
}

/// アドバタイズしてGATT接続を待つ
async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"InvertedPendulum"),
        ],
        &mut adv_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[ble] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[ble] connection established");
    Ok(conn)
}

/// GATTイベント処理: Write受信 → コマンドチャネルへ転送
async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let command_handle = server.pendulum_service.command;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Write(ev) => {
                        if ev.handle() == command_handle.handle {
                            let data = ev.data();
                            if data.len() >= 3 {
                                if let Some(cmd_type) = CommandType::from_u8(data[0]) {
                                    let cmd = inverted_pendulum_protocol::Command {
                                        command_type: cmd_type,
                                        payload: [data[1], data[2]],
                                    };
                                    info!("[ble] command received: {:?}", cmd);
                                    COMMAND_CHANNEL.send(cmd).await;
                                } else {
                                    warn!("[ble] unknown command type: {}", data[0]);
                                }
                            }
                        }
                    }
                    _ => {}
                };
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[ble] error sending response: {:?}", e),
                };
            }
            _ => {}
        }
    };
    info!("[ble] disconnected: {:?}", reason);
    Ok(())
}

/// センサデータ通知タスク: BLE_SENSOR_CHANNELからデータを受け取り、BLE notifyで送信
async fn notify_task<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let sensor_char = server.pendulum_service.sensor_data;
    loop {
        let data = BLE_SENSOR_CHANNEL.receive().await;
        let mut buf = [0u8; SENSOR_DATA_RAW_SIZE];
        if data.serialize(&mut buf).is_ok() {
            if sensor_char.notify(conn, &buf).await.is_err() {
                info!("[ble] notify error, connection may be closed");
                break;
            }
        }
    }
}
