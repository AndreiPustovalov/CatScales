#![no_std]
#![no_main]

use cortex_m_rt::{exception, ExceptionFrame};
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts, gpio,
    interrupt::{self, typelevel::Interrupt as _},
    peripherals,
    usb::{self},
    twim
};
use embassy_time::{with_timeout, Delay, Duration, Timer, TimeoutError};
use nrf_softdevice::{Softdevice, ble};
use embedded_hal::digital::{InputPin, OutputPin};
use loadcell::{hx711::HX711, LoadCell};
use lis3dh_async::{Lis3dh, SlaveAddr, Mode, DataRate, Range, IrqPin1Config, Interrupt1, InterruptMode, InterruptConfig, LatchInterruptRequest, Detect4D, Threshold, Duration as Lis3dhDuration, Error};
use embedded_hal;
use rtt_target::rtt_init_log;

const SCALE: f32 = 1.0/57156.0;
bind_interrupts!(
    struct Irqs {
        USBD => usb::InterruptHandler<peripherals::USBD>;
        TWISPI1 => twim::InterruptHandler<peripherals::TWISPI1>;
    }
);

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    let p0 = embassy_nrf::pac::P0;
    loop {
        p0.outset().write(|w| { w.set_pin(26, true) });
        cortex_m::asm::delay(1_000_000);
        p0.outclr().write(|w| { w.set_pin(26, true) });
        cortex_m::asm::delay(1_000_000);
    }
}

#[exception]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    let p0 = embassy_nrf::pac::P0;

    loop {
        p0.outclr().write(|w| { w.set_pin(26, true) });
        cortex_m::asm::delay(1_000_000);
        p0.outset().write(|w| { w.set_pin(26, true) });
        cortex_m::asm::delay(1_000_000);
        p0.outclr().write(|w| { w.set_pin(26, true) });
        cortex_m::asm::delay(1_000_000);
        p0.outset().write(|w| { w.set_pin(26, true) });
        cortex_m::asm::delay(5_000_000);
    }
}

type Accel = Lis3dh<lis3dh_async::Lis3dhI2C<twim::Twim<'static>>>;

async fn init_accelerometer<E>(i2c: twim::Twim<'static>) -> Result<Accel, Error<E>>
where
    Error<E>: From<Error<twim::Error>>
{
    let mut lis3dh = Lis3dh::new_i2c(i2c, SlaveAddr::Default).await?;
    let range = Range::G2;          // load-cell platform motion is coarse; ±2g is plenty
    let data_rate = DataRate::Hz_10; // lowest ODR = lowest current in low-power mode

    // LowPower mode (8-bit resolution) + low ODR is the chip's minimum-current combo —
    // per the datasheet this is roughly single-digit µA at 1 Hz, vs ~mA-range in
    // Normal/HighResolution mode. Confirm the exact figure against your datasheet revision.
    lis3dh.set_mode(Mode::LowPower).await?;
    lis3dh.set_range(range).await?;
    lis3dh.set_datarate(data_rate).await?;

    // Trigger on motion above threshold, on any axis, either direction.
    let threshold = Threshold::g(range, 1.05); // tune: lower = more sensitive/false triggers
    // let duration = Lis3dhDuration::miliseconds(data_rate, 1.0); // fire immediately, no min dwell
    let duration = Lis3dhDuration::ZERO;

    lis3dh.configure_irq_src_and_control(
        Interrupt1,
        InterruptMode::Movement,
        InterruptConfig::high_and_low(),
        LatchInterruptRequest::Enable, // hold INT1 high until we read IRQ src — don't miss short events
        Detect4D::Disable,
    ).await?;
    lis3dh.configure_irq_duration(Interrupt1, duration).await?;
    lis3dh.configure_irq_threshold(Interrupt1, threshold).await?;

    // Route interrupt 1 to the INT1 pin.
    lis3dh.configure_interrupt_pin(IrqPin1Config {
        ia1_en: true,
        ..IrqPin1Config::default()
    }).await?;

    lis3dh.get_irq_src(Interrupt1).await?;

    Ok(lis3dh)
}

/// Resets the device into Device Firmware Update mode (DFU).
fn reset_into_dfu() -> ! {
    // Via https://github.com/adafruit/Adafruit_nRF52_Bootloader#how-to-use
    // This should allow us to reset into DFU/serial bootloader mode after reset.

    // Bootloader: enter CDC/serial DFU on next reset.
    const GPREGRET_ENTER_SERIAL_DFU: u8 = 0x4E;
    // Bootloader: enter UF2 + CDC bootloader on next reset.
    #[allow(dead_code)]
    const GPREGRET_ENTER_UF2_DFU: u8 = 0x57;
    // Bootloader: enter OTA DFU mode on next reset.
    #[allow(dead_code)]
    const GPREGRET_ENTER_OTA_DFU: u8 = 0xA8;

    // Clear GPREGRET then set exact bootloader value.
    unsafe {
        nrf_softdevice::raw::sd_power_gpregret_clr(0, 0xff);
        nrf_softdevice::raw::sd_power_gpregret_set(0, GPREGRET_ENTER_SERIAL_DFU as u32);
    }
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::task]
async fn softdevice_task(
    sd: &'static Softdevice,
    vbus: &'static usb::vbus_detect::SoftwareVbusDetect,
) -> ! {
    sd.run_with_callback(|event| {
        use nrf_softdevice::SocEvent;

        // Forward USB events.
        match event {
            SocEvent::PowerUsbDetected => vbus.detected(true),
            SocEvent::PowerUsbRemoved => vbus.detected(false),
            SocEvent::PowerUsbPowerReady => vbus.ready(),
            _ => {}
        }
    })
        .await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_init_log!();

    // Per https://github.com/embassy-rs/nrf-softdevice/tree/nrf-softdevice-v0.1.0#interrupt-priority
    // Interrupt priorities 0, 1 and 4 are reserved by the Softdevice, so we have to use 2 or 3 for all interrupts.
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    config.gpiote_interrupt_priority = interrupt::Priority::P2;
    config.time_interrupt_priority = interrupt::Priority::P2;
    let peripherals = embassy_nrf::init(config);
    interrupt::typelevel::USBD::set_priority(interrupt::Priority::P2);
    interrupt::typelevel::TWISPI0::set_priority(interrupt::Priority::P3);
    interrupt::typelevel::TWISPI1::set_priority(interrupt::Priority::P3);

    let config = softdevice_config();
    let sd = Softdevice::enable(&config);

    let _led_red = gpio::Output::new(peripherals.P0_26, gpio::Level::High, gpio::OutputDrive::Standard); // unused
    let _led_green = gpio::Output::new(peripherals.P0_30, gpio::Level::High, gpio::OutputDrive::Standard); // unused
    let led_blue = gpio::Output::new(peripherals.P0_06, gpio::Level::High, gpio::OutputDrive::Standard);

    let mut config = twim::Config::default();
    config.frequency = twim::Frequency::K100; // bump to K400 once it's working reliably
    config.sda_pullup = true;
    config.scl_pullup = true;

    let i2c = twim::Twim::new(
        peripherals.TWISPI1,
        Irqs,
        peripherals.P1_15 /*SDA*/,
        peripherals.P1_14 /*SCL*/,
        config,
        &mut[]
    );

    let mut a = match init_accelerometer(i2c).await {
        Ok(a) => {
            log::info!("WHO_AM_I = {:?}", 0);
            a
        },
        Err(e) => {
            log::error!("Error: {:?}", e);
            panic!("Error: {:?}", e);
        },
    };

    let mut int1_pin = gpio::Input::new(peripherals.P0_28, gpio::Pull::None); // accelerometer interrupt

    let sck_pin = gpio::Output::new(peripherals.P0_02, gpio::Level::Low, gpio::OutputDrive::Standard);
    let dout_pin = gpio::Input::new(peripherals.P0_03, gpio::Pull::Up);

    let mut load_sensor = HX711::new(sck_pin, dout_pin, Delay);
    load_sensor.set_scale(SCALE);

    tare(&mut load_sensor, 5).await;
    spawner.spawn(blink_task(led_blue).unwrap());
    loop {
        if int1_pin.is_high() {
            a.get_irq_src(Interrupt1).await.expect("Couldn't get irq src");
        }
        match with_timeout(Duration::from_secs(60), int1_pin.wait_for_rising_edge()).await {
            Ok(_) => {
                match a.get_irq_src(Interrupt1).await {
                    Ok(src) => log::info!("Edge detected: {:?}. Waiting", src),
                    Err(e) => log::error!("Error waiting edge: {:?}", e)
                }
            },
            Err(e) => match e {
                TimeoutError => log::info!("Timeout")
            },
        }
        Timer::after(Duration::from_secs(5)).await;

        if let Some((raw, kg)) = read_weight(&mut load_sensor).await {
            setup_ble_advertising(sd, kg, raw, 90).await;
            log::info!("Sent: {} kg, {} raw", kg, raw);
        }
    }
}

#[embassy_executor::task]
async fn blink_task(mut led: gpio::Output<'static>) {
    for _ in 0..2 {
        led.set_low();
        Timer::after(Duration::from_millis(200)).await;
        led.set_high();
        Timer::after(Duration::from_millis(300)).await;
    }
}

async fn read_weight<SckPin, DTPin>(
    load_sensor: &mut HX711<SckPin, DTPin, Delay>,
) -> Option<(i32, f32)>
where
    SckPin: OutputPin,
    DTPin: InputPin
{
    while !load_sensor.is_ready() {
        Timer::after(Duration::from_millis(10)).await;
    }
    match load_sensor.read() {
        Ok(raw) => Some((raw, raw as f32 * load_sensor.get_scale())),
        Err(e) => {log::error!("Error: {:?}", e); None}
    }
}

async fn tare<SckPin, DTPin>(load_sensor: &mut HX711<SckPin, DTPin, Delay>, num_samples: u8)
where
    SckPin: OutputPin,
    DTPin: InputPin
{
    let mut average: f32 = 0.0;
    for _ in 1..=num_samples {
        while !load_sensor.is_ready() {
            Timer::after(Duration::from_millis(10)).await;
        }
        average += load_sensor.read().unwrap() as f32;
        Timer::after(Duration::from_millis(50)).await;
    }

    load_sensor.set_offset((average / num_samples as f32) as i32);
}

fn build_bthome_payload(weight_kg: f32, weight_raw: i32, battery_pct: u8) -> [u8; 19] {
    let weight_converted = (weight_kg * 100.0) as u16;

    [
        // --- 1. GAP Flags (3 bytes) ---
        0x02, 0x01, 0x06,
        // --- 2. BTHome V2 Service Data (10 bytes) ---
        0x0F,       // Structure length
        0x16,       // AD Type: Service Data 16-bit UUID
        0xD2, 0xFC, // BTHome UUID 0xFCD2 (Little-Endian)
        0x40,       // BTHome V2 Header (Unencrypted)
        // --- 3.
        0x06,       // Sensor ID: Mass (0x06)
        weight_converted as u8,
        (weight_converted >> 8) as u8,
        // --- 4.
        0x01,       // Sensor ID: Battery % (0x01)
        battery_pct,
        0x54,
        0x04,
        weight_raw as u8,
        (weight_raw >> 8) as u8,
        (weight_raw >> 16) as u8,
        (weight_raw >> 24) as u8,
    ]
}

async fn setup_ble_advertising(sd: &'static Softdevice, weight: f32, weight_raw: i32, battery: u8) {
    let adv_data = build_bthome_payload(weight, weight_raw, battery);
    let scan_data = [
        0x0A,       // Structure length: 1 byte (type) + 9 bytes ("Cat Scale") = 10 (0x0A)
        0x09,       // AD Type: Complete Local Name (0x09)
        b'C', b'a', b't', b' ', b'S', b'c', b'a', b'l', b'e'
    ];

    // Configure non-connectable, fast broadcasting payload
    let config = ble::peripheral::Config::default();
    let adv = ble::peripheral::NonconnectableAdvertisement::ScannableUndirected {
        adv_data: &adv_data,
        scan_data: &scan_data,
    };

    let _ = with_timeout(
        Duration::from_secs(2),
        ble::peripheral::advertise(sd, adv, &config)
    ).await;
}

fn softdevice_config() -> nrf_softdevice::Config {
    use nrf_softdevice::raw;

    let name = b"CatScales";

    nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8, // TODO: switch to external? NRF_CLOCK_LF_SRC_XTAL?
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        // Configure GAP (Generic Access Profile) connection resource.
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 24,
        }),
        // Configure GATT (Generic Attribute Profile) connection resource.
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }), // Bumps up the maximum transmission unit, allowing us to send more data in one packet.
        // Attribute table size.
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        // Configure BLE roles.
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,  //raw::BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT as _,
            central_role_count: 3, //raw::BLE_GAP_ROLE_COUNT_CENTRAL_DEFAULT as _,
            central_sec_count: 0,  //raw::BLE_GAP_ROLE_COUNT_CENTRAL_SEC_DEFAULT as _,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        // Configure GAP (Generic Access Profile) device name.
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: name.as_ptr() as *const u8 as _,
            current_len: name.len() as _,
            max_len: name.len() as _,
            write_perm: unsafe { core::mem::zeroed() }, // Not writable.
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    }
}
