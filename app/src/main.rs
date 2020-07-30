#![feature(never_type)]
#![cfg_attr(not(doc), no_main)]

use std::io::prelude::*;
use std::net::TcpStream;
use std::net::{Ipv4Addr, SocketAddrV4};
use std::sync::{Arc, Mutex};
use std::thread::{self, sleep};
use std::time::Duration;

use embedded_hal::digital::v2::OutputPin;
use macaddr::MacAddr;

use esp_idf_hal::{interface::*, nvs::*, wifi::*, *};

use futures::executor::block_on;

use {
    bytes::BytesMut,
    embedded_graphics::{
        fonts::{Font6x8, Text},
        pixelcolor::BinaryColor,
        prelude::*,
        style::TextStyleBuilder,
    },
    mqttrs::{Connect, Packet, PacketType, Pid, Protocol, QoS, Subscribe, SubscribeTopic},
    sgp30::Sgp30,
    ssd1306::{mode::GraphicsMode, prelude::*, Builder},
};

mod wifi_manager;
use wifi_manager::*;

mod dns;

#[no_mangle]
pub fn app_main() {
    block_on(async {
        if let Err(err) = rust_blink_and_write().await {
            println!("{}", err);
        }
    })
}

async fn rust_blink_and_write() -> Result<!, EspError> {
    use esp32_hal::{gpio::GpioExt, target};

    let dp = unsafe { target::Peripherals::steal() };
    let pins = dp.GPIO.split();

    let mut led = pins.gpio25.into_push_pull_output();

    let mut nvs = NonVolatileStorage::default();

    let text_style = TextStyleBuilder::new(Font6x8)
        .text_color(BinaryColor::On)
        .build();

    /*let mut disp = {
        let i2c = unsafe {
            i2c::Master::new(
                i2c::Port::Port0,
                i2c::PinConfig {
                    pin_num: 4,
                    pullup: true,
                },
                i2c::PinConfig {
                    pin_num: 15,
                    pullup: true,
                },
                200_000,
            )
            .unwrap()
        };

        let mut disp: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();

        let mut rst = pins.gpio16.into_push_pull_output();
        rst.set_low().unwrap();
        sleep(Duration::from_millis(10));
        rst.set_high().unwrap();

        disp.init().unwrap();

        Text::new("MQTT client starting!", Point::zero())
            .into_styled(text_style)
            .draw(&mut disp);

        disp.flush().unwrap();

        disp
    };*/

    let mut sgp = {
        let i2c = unsafe {
            i2c::Master::new(
                i2c::Port::Port0,
                i2c::PinConfig {
                    pin_num: 22,
                    pullup: true,
                },
                i2c::PinConfig {
                    pin_num: 23,
                    pullup: true,
                },
                200_000,
            )
            .unwrap()
        };

        let mut sgp = Sgp30::new(i2c, 0x58, Delay);

        sgp.init().unwrap();
        dbg!(sgp.measure().unwrap());

        sgp
    };

    let wifi = Wifi::take().unwrap();

    let namespace = nvs.namespace("wifi")?;
    println!("namespace: {:?}", namespace);

    thread::Builder::new()
        .name("dns_thread".into())
        .stack_size(6144)
        .spawn(dns::server)
        .unwrap();

    thread::Builder::new()
        .name("main_thread".into())
        .stack_size(8192)
        .spawn(move || {
            block_on(async {
                let mac = MacAddr::from(Interface::Ap);

                let ap_ssid = Ssid::from_bytes(format!("ESP {}", mac).as_bytes()).unwrap();

                let ap_config = ApConfig::builder().ssid(ap_ssid).build();

                let wifi_storage = namespace;

                let ssid = Ssid::from_bytes("".as_bytes()).unwrap();
                let password = Password::from_bytes("".as_bytes()).unwrap();

                let wifi_running =
                    wifi_manager::connect_ssid_password(wifi, ap_config, ssid, password).await;

                let mut stream = TcpStream::connect("2554d17.local:1883").unwrap();

                stream
                    .set_read_timeout(Some(Duration::from_millis(500)))
                    .unwrap();

                let response = {
                    let mut buf = BytesMut::with_capacity(128);

                    mqttrs::encode(
                        &Packet::Connect(Connect {
                            protocol: Protocol::MQTT311,
                            keep_alive: 30,
                            client_id: "doc_client".into(),
                            clean_session: true,
                            last_will: None,
                            username: None,
                            password: None,
                        }),
                        &mut buf,
                    )
                    .unwrap();

                    stream.write(&buf).unwrap();
                    stream.read(&mut buf).unwrap();
                    dbg!(mqttrs::decode(&mut buf)).unwrap().unwrap()
                };

                let response = {
                    let mut buf = BytesMut::with_capacity(128);

                    mqttrs::encode(
                        &Packet::Subscribe(Subscribe {
                            pid: Pid::new(),
                            topics: vec![SubscribeTopic {
                                topic_path: "switch/seton".to_string(),
                                qos: QoS::AtMostOnce,
                            }],
                        }),
                        &mut buf,
                    )
                    .unwrap();

                    stream.write(&buf).unwrap();
                    stream.read(&mut buf).unwrap();
                    dbg!(mqttrs::decode(&mut buf).unwrap()).unwrap()
                };

                let stream = Arc::new(Mutex::new(stream));

                {
                    let stream = stream.clone();
                    thread::Builder::new()
                        .name("mqtt_main_thread".into())
                        .stack_size(4096)
                        .spawn(move || block_on(mqtt_main(stream, led)))
                        .unwrap();
                }

                let mut buf = BytesMut::with_capacity(32);
                loop {
                    mqttrs::encode(&Packet::Pingreq, &mut buf).unwrap();

                    {
                        let mut stream = stream.lock().unwrap();
                        stream.write(&buf).unwrap();
                        stream.read(&mut buf).unwrap();
                    }

                    let pkt = mqttrs::decode(&mut buf).unwrap().unwrap();
                    buf.clear();

                    assert_eq!(pkt.get_type(), PacketType::Pingresp);

                    println!("ping ok");

                    sleep(Duration::from_secs(15))
                }
            })
        })
        .unwrap();

    loop {
        sleep(Duration::from_secs(30))
    }
}

async fn mqtt_main(
    stream: Arc<Mutex<TcpStream>>,
    mut led: esp32_hal::gpio::Gpio25<esp32_hal::gpio::Output<esp32_hal::gpio::PushPull>>,
) {
    let mut buf = [0u8; 128];
    loop {
        {
            let mut stream = stream.lock().unwrap();
            match stream.read(&mut buf) {
                Ok(n) => {
                    let pkt = mqttrs::decode(&mut BytesMut::from(&buf[..]))
                        .unwrap()
                        .unwrap();

                    match pkt {
                        Packet::Publish(publish) => match publish.payload[..] {
                            [48] => {
                                println!("OFF");
                                led.set_low().unwrap();
                            }
                            [49] => {
                                println!("ON");
                                led.set_high().unwrap();
                            }
                            _ => panic!("{:?}", publish),
                        },
                        p => {
                            dbg!(p);
                        }
                    }

                    for i in 0..n {
                        buf[i] = 0;
                    }
                }
                Err(e) => match e.kind() {
                    std::io::ErrorKind::WouldBlock => {}
                    _ => {
                        dbg!(e);
                    }
                },
            }
        }
        sleep(Duration::from_millis(10));
    }
}

struct Delay;

impl embedded_hal::blocking::delay::DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        sleep(Duration::from_micros(us.into()));
    }
}

impl embedded_hal::blocking::delay::DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        sleep(Duration::from_millis(ms.into()));
    }
}
