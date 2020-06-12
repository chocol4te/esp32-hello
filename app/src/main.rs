#![feature(never_type)]
#![no_main]

extern crate alloc;

use alloc::string::String;

#[macro_use]
extern crate std;

use std::net::TcpListener;
use std::net::{Ipv4Addr, SocketAddrV4};
use std::sync::{Mutex, RwLock};
use std::thread::{self, sleep};
use std::time::Duration;

use embedded_hal::digital::v2::OutputPin;

use esp32_hal::{gpio::*, nvs::*, wifi::*, *};

use futures::executor::block_on;

use embedded_graphics::{
    fonts::{Font12x16, Text},
    pixelcolor::BinaryColor,
    prelude::*,
    style::TextStyleBuilder,
};
use ssd1306::{prelude::*, Builder};

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

use std::cell::RefCell;
thread_local! {
  pub static FOO: RefCell<u32> = RefCell::new(0);
}

async fn rust_blink_and_write() -> Result<!, EspError> {
    let mut gpio = GPIO25::into_input_output();

    let mut nvs = NonVolatileStorage::default()?;

    let wifi = Wifi::init(&mut nvs);

    println!("AP started.");

    FOO.with(|f| {
        *f.borrow_mut() += 1;
    });

    thread::spawn(|| {
        FOO.with(|f| {
            *f.borrow_mut() += 1;
        });

        FOO.with(|f| {
            println!("THREAD 1: {:?}", f.borrow());
        });
    });

    thread::spawn(|| {
        FOO.with(|f| {
            *f.borrow_mut() += 1;
        });

        FOO.with(|f| {
            println!("THREAD 2: {:?}", f.borrow());
        });
    });

    FOO.with(|f| {
        println!("MAIN THREAD: {:?}", f.borrow());
    });

    // esp32_hal::wifi::wifi_scan(true, false, 1000)?;

    let mutex = Mutex::new(0usize);
    println!("mutex value = {:?}", *mutex.lock().unwrap());
    *mutex.lock().unwrap() = 1;
    println!("mutex value = {:?}", *mutex.lock().unwrap());

    let rwlock = RwLock::new(0usize);
    println!("rwlock value = {:?}", *rwlock.read().unwrap());
    *rwlock.write().unwrap() = 1;
    println!("rwlock value = {:?}", *rwlock.read().unwrap());

    let namespace = nvs.open("wifi")?;
    println!("namespace: {:?}", namespace);

    let t = thread::Builder::new()
        .name("hello_thread".into())
        .stack_size(8192)
        .spawn(|| {
            println!("HELLO, WORLD!");
            42
        });

    println!("Thread spawn result: {:?}", t);
    println!("Thread join result: {:?}", t.map(|t| t.join().unwrap()));

    thread::Builder::new()
        .name("dns_thread".into())
        .stack_size(8192)
        .spawn(dns::server)
        .unwrap();

    thread::Builder::new()
        .name("blink_thread".into())
        .spawn(move || loop {
            gpio.set_low().unwrap();
            sleep(Duration::from_millis(100));
            gpio.set_high().unwrap();
            sleep(Duration::from_secs(1));
        })
        .unwrap();

    thread::Builder::new()
        .name("server_thread".into())
        .stack_size(8192)
        .spawn(move || {
            block_on(async {
                let mac = mac_address(MacAddressType::Ap);
                let ap_ssid = format!("ESP {}", mac);

                let ap_config = ApConfig::builder().ssid(&ap_ssid).build();

                let mut wifi_storage = namespace;

                let ssid = wifi_storage.get::<String>("ssid").ok();
                let password = wifi_storage.get::<String>("password").ok();

                let mut ap_running = None;
                let mut sta_running = None;

                if let (Some(ssid), Some(password)) = (ssid, password) {
                    let sta_config = StaConfig::builder().ssid(&ssid).password(&password).build();

                    let sta = wifi.into_sta(&sta_config);

                    match sta.connect().await {
                        Ok(sta) => {
                            let StaRunning(ip) = sta;
                            println!("Connected to '{}' with IP '{}'.", ssid, Ipv4Addr::from(ip));
                            sta_running = Some(sta);
                        }
                        Err(err) => {
                            let ap = err.wifi().into_ap(&ap_config);
                            ap_running = Some(ap.start());
                        }
                    }
                } else {
                    println!("Starting Access Point '{}' …", ap_ssid);
                    let ap = wifi.into_ap(&ap_config);
                    ap_running = Some(ap.start());
                }

                let stream = TcpListener::bind(SocketAddrV4::new(Ipv4Addr::new(0, 0, 0, 0), 80))
                    .expect("failed starting TCP listener");

                loop {
                    match stream.accept() {
                        Ok((client, addr)) => {
                            match handle_request(
                                client,
                                addr,
                                &ap_config,
                                &mut wifi_storage,
                                ap_running.take(),
                                sta_running.take(),
                            )
                            .await
                            {
                                Ok((ap, sta)) => {
                                    ap_running = ap;
                                    sta_running = sta;
                                }
                                Err(err) => {
                                    eprintln!("Failed to handle request: {:?}", err);
                                }
                            }
                        }
                        Err(e) => {
                            if e.kind() != std::io::ErrorKind::WouldBlock {
                                eprintln!("couldn't get client: {:?}", e);
                            }
                        }
                    }

                    thread::yield_now();
                }
            })
        })
        .unwrap();

    let mut dport = unsafe { target::Peripherals::steal().DPORT };

    //i2c_set_pin(0, 4, 15, true, true, i2c_mode_t::I2C_MODE_MASTER);
    let gpio = unsafe { &*target::GPIO::ptr() };
    let iomux = unsafe { &*target::IO_MUX::ptr() };

    // sda
    {
        // gpio_set_level(sda_io_num, I2C_IO_INIT_LEVEL);
        gpio.pin4.write(|w| unsafe { w.bits(1 << 4) });

        // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[sda_io_num], PIN_FUNC_GPIO);
        iomux.gpio4.modify(|_, w| unsafe { w.mcu_sel().bits(2) });

        // gpio_set_direction(sda_io_num, GPIO_MODE_INPUT_OUTPUT_OD);
        iomux
            .gpio4
            .modify(|_, w| w.fun_ie().set_bit().mcu_oe().set_bit());

        // gpio_set_pull_mode(sda_io_num, GPIO_PULLUP_ONLY);
        iomux
            .gpio4
            .modify(|_, w| w.fun_wpd().clear_bit().fun_wpu().set_bit());
    }

    // scl
    {
        // gpio_set_level(scl_io_num, I2C_IO_INIT_LEVEL);
        gpio.pin15.write(|w| unsafe { w.bits(1 << 15) });

        // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[scl_io_num], PIN_FUNC_GPIO);
        iomux.mtdo.modify(|_, w| unsafe { w.mcu_sel().bits(2) });

        // gpio_set_direction(scl_io_num, GPIO_MODE_INPUT_OUTPUT_OD);
        iomux
            .mtdo
            .modify(|_, w| w.fun_ie().set_bit().mcu_oe().set_bit());

        // gpio_set_pull_mode(scl_io_num, GPIO_PULLUP_ONLY);
        iomux
            .mtdo
            .modify(|_, w| w.fun_wpd().clear_bit().fun_wpu().set_bit());
    }

    let i2c = i2c::I2C::new(
        unsafe { target::Peripherals::steal().I2C0 },
        i2c::PinConfig {
            pin_num: 4,
            pullup: true,
        },
        i2c::PinConfig {
            pin_num: 15,
            pullup: true,
        },
        400_000,
        &mut dport,
    );

    let mut disp = Builder::new().connect_i2c(i2c).into::<_, GraphicsMode<_>>();

    let mut rst = GPIO16::into_input_output();
    rst.set_high().unwrap();
    sleep(Duration::from_millis(1));
    rst.set_low().unwrap();
    sleep(Duration::from_millis(10));
    rst.set_high().unwrap();

    disp.init().unwrap();

    disp.clear();
    disp.flush().unwrap();

    let text_style = TextStyleBuilder::new(Font12x16)
        .text_color(BinaryColor::On)
        .build();

    loop {
        disp.clear();
        Text::new("ESP32 I2C", Point::zero())
            .into_styled(text_style)
            .draw(&mut disp)
            .unwrap();
        Text::new("in Rust!", Point::new(0, 18))
            .into_styled(text_style)
            .draw(&mut disp)
            .unwrap();
        Text::new("tick", Point::new(0, 48))
            .into_styled(text_style)
            .draw(&mut disp)
            .unwrap();
        disp.flush().unwrap();
        sleep(Duration::from_millis(500));

        disp.clear();
        Text::new("ESP32 I2C", Point::zero())
            .into_styled(text_style)
            .draw(&mut disp)
            .unwrap();
        Text::new("in Rust!", Point::new(0, 18))
            .into_styled(text_style)
            .draw(&mut disp)
            .unwrap();
        Text::new("tock", Point::new(64, 48))
            .into_styled(text_style)
            .draw(&mut disp)
            .unwrap();
        disp.flush().unwrap();
        sleep(Duration::from_millis(500));
    }
}
