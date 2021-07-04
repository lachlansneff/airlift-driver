use core::{
    array,
    convert::Infallible,
    ops::{Deref, DerefMut},
    slice, str,
};

use embedded_hal::{
    blocking::{delay::DelayMs, spi},
    digital::v2::{InputPin, OutputPin},
};

const REPLY_FLAG: u8 = 1 << 7;
const START_CMD: u8 = 0xe0;
const END_CMD: u8 = 0xee;
const ERR_CMD: u8 = 0xef;

#[derive(Debug)]
pub enum Error<E> {
    Other(E),
    WrongValue { expected: u8, got: u8 },
    BufferTooSmall { was: usize, needed: usize },
    Timeout,
    InvalidEncoding,
    CmdFailed(Cmd),
    ConnectionFailed,
    NoSuchSSID,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Other(e)
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cmd {
    SetNet = 0x10,
    SetPassPhrase = 0x11,
    SetKey = 0x12,
    SetIPConfig = 0x14,
    SetDNSConfig = 0x15,
    SetHostname = 0x16,
    SetPowerMode = 0x17,
    SetAPNet = 0x18,
    SetAPPassPhrase = 0x19,
    SetDebug = 0x1a,
    GetTemperature = 0x1b,
    GetConnStatus = 0x20,
    GetIPAddress = 0x21,
    GetMACAddress = 0x22,
    GetCurrentSSID = 0x23,
    GetCurrentBSSID = 0x24,
    GetCurrentRSSI = 0x25,
    GetCurrentEncryption = 0x26,
    ScanNetwork = 0x27,
    StartServerTCP = 0x28,
    GetStateTCP = 0x29,
    DataSentTCP = 0x2a,
    AvailableDataTCP = 0x2b,
    GetDataTCP = 0x2c,
    StartClientTCP = 0x2d,
    StopClientTCP = 0x2e,
    GetClientStateTCP = 0x2f,
    Disconnect = 0x30,
    GetIndexRSSI = 0x32,
    GetIndexEncryption = 0x33,
    RequestHostByName = 0x34,
    GetHostByName = 0x35,
    StartScanNetworks = 0x36,
    GetFirmwareVersion = 0x37,
    SendUDPData = 0x39,
    GetRemoteData = 0x3a,
    GetTime = 0x3b,
    GetIndexBSSID = 0x3c,
    GetIndexChannel = 0x3d,
    Ping = 0x3e,
    GetSocket = 0x3f,
    SetClientCert = 0x40,
    SetCertKey = 0x41,
    SendDataTCP = 0x44,
    GetDataBufTCP = 0x45,
    InsertDataBuf = 0x46,
    WPA2EnterpriseSetIdentity = 0x4a,
    WPA2EnterpriseSetUsername = 0x4b,
    WPA2EnterpriseSetPassword = 0x4c,
    WPA2EnterpriseSetCACert = 0x4d,
    WPA2EnterpriseSetCertKey = 0x4e,
    WPA2EnterpriseEnable = 0x4f,
    SetPinMode = 0x50,
    SetDigitalWrite = 0x51,
    SetAnalogWrite = 0x52,
    SetDigitalRead = 0x53,
    SetAnalogRead = 0x54,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ParamLen {
    U8,
    U16,
}

impl Default for ParamLen {
    fn default() -> Self {
        ParamLen::U8
    }
}

#[derive(Default)]
struct SendConfig {
    param_len: ParamLen,
}

#[derive(Default)]
struct ReceiveConfig {
    param_len: ParamLen,
    allow_fewer_resp: bool,
}

#[derive(Default)]
struct SendReceiveConfig {
    send: SendConfig,
    receive: ReceiveConfig,
}

struct SpiGuard<'a, SPI, CS: OutputPin<Error = Infallible>> {
    spi: &'a mut SPI,
    cs: &'a mut CS,
}

impl<'a, SPI, CS: OutputPin<Error = Infallible>> SpiGuard<'a, SPI, CS> {
    fn new(spi: &'a mut SPI, cs: &'a mut CS) -> Self {
        cs.set_low().ok();
        Self { spi, cs }
    }
}

impl<'a, SPI, CS: OutputPin<Error = Infallible>> Deref for SpiGuard<'a, SPI, CS> {
    type Target = SPI;

    fn deref(&self) -> &Self::Target {
        &self.spi
    }
}

impl<'a, SPI, CS: OutputPin<Error = Infallible>> DerefMut for SpiGuard<'a, SPI, CS> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.spi
    }
}

impl<'a, SPI, CS: OutputPin<Error = Infallible>> Drop for SpiGuard<'a, SPI, CS> {
    fn drop(&mut self) {
        self.cs.set_high().ok();
    }
}

struct Rest<'a, TIMER> {
    // busy: &'a mut BUSY,
    // reset: &'a mut RESET,
    // gpio0: &'a mut GPIO0,
    timer: &'a mut TIMER,
}

struct Driver<SPI, CS, BUSY, RESET, GPIO0, TIMER> {
    spi: SPI,
    cs: CS,
    busy: BUSY,
    reset: RESET,
    gpio0: GPIO0,

    timer: TIMER,
}

impl<SPI, CS, BUSY, RESET, GPIO0, TIMER, E> Driver<SPI, CS, BUSY, RESET, GPIO0, TIMER>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    CS: OutputPin<Error = Infallible>,
    BUSY: InputPin<Error = Infallible>,
    RESET: OutputPin<Error = Infallible>,
    GPIO0: OutputPin<Error = Infallible>,
    TIMER: DelayMs<u32>,
{
    fn guard(&mut self) -> (SpiGuard<SPI, CS>, Rest<TIMER>) {
        (
            SpiGuard::new(&mut self.spi, &mut self.cs),
            Rest {
                // busy: &mut self.busy,
                // reset: &mut self.reset,
                // gpio0: &mut self.gpio0,
                timer: &mut self.timer,
            },
        )
    }

    pub fn reset(&mut self) -> Result<(), E> {
        self.gpio0.set_high().ok();
        self.cs.set_high().ok();
        self.reset.set_low().ok();

        // delay 10ms, reset
        self.timer.delay_ms(10);

        self.reset.set_high().ok();

        // delay 750ms, wait for it to boot up
        self.timer.delay_ms(750);

        Ok(())
    }

    /// Wait until busy pin goes low
    fn wait_for_busy(&mut self) -> Result<(), Error<E>> {
        let mut count = 0;

        loop {
            if let Ok(true) = self.busy.is_low() {
                break Ok(());
            }

            self.timer.delay_ms(1);
            count += 1;

            if count == 10_000 {
                break Err(Error::Timeout);
            }
        }
    }

    fn send_cmd<const PARAMS: usize>(
        &mut self,
        cmd: Cmd,
        params: [&[u8]; PARAMS],
        config: SendConfig,
    ) -> Result<(), Error<E>> {
        assert!(PARAMS <= u8::MAX as usize);

        self.wait_for_busy()?;

        let (mut spi, _) = self.guard();

        spi.write(&[START_CMD, cmd as u8 & !REPLY_FLAG, params.len() as u8])?;
        for param in params {
            match config.param_len {
                ParamLen::U8 => {
                    assert!(param.len() <= u8::MAX as usize);
                    spi.write(&[param.len() as u8])?;
                }
                ParamLen::U16 => {
                    assert!(param.len() <= u16::MAX as usize);
                    spi.write(&[(param.len() >> 8) as u8, params.len() as u8])?;
                }
            }

            spi.write(param)?;
        }

        spi.write(&[END_CMD])?;

        Ok(())
    }

    fn wait_spi_byte(
        spi: &mut SpiGuard<SPI, CS>,
        timer: &mut TIMER,
        expected: u8,
    ) -> Result<(), Error<E>> {
        let mut count = 0;

        loop {
            timer.delay_ms(1);
            count += 1;

            let mut b = 0xff;
            spi.transfer(slice::from_mut(&mut b))?;

            if b == ERR_CMD {
                return Err(Error::WrongValue { expected, got: b });
            }

            if b == expected {
                return Ok(());
            }

            if count == 1000 {
                return Err(Error::Timeout);
            }
        }
    }

    fn check_spi_byte(spi: &mut SpiGuard<SPI, CS>, expected: u8) -> Result<(), Error<E>> {
        let mut b = 0xff;
        spi.transfer(slice::from_mut(&mut b))?;

        if b == expected {
            Ok(())
        } else {
            Err(Error::WrongValue { expected, got: b })
        }
    }

    fn receive_cmd<'a, const RESPONSES: usize>(
        &mut self,
        cmd: Cmd,
        responses: [&'a mut [u8]; RESPONSES],
        config: ReceiveConfig,
    ) -> Result<heapless::Vec<&'a [u8], RESPONSES>, Error<E>> {
        assert!(RESPONSES <= u8::MAX as usize);
        self.wait_for_busy()?;

        let (mut spi, rest) = self.guard();

        Self::wait_spi_byte(&mut spi, rest.timer, START_CMD)?;
        Self::check_spi_byte(&mut spi, cmd as u8 | REPLY_FLAG)?;

        let response_num = if config.allow_fewer_resp {
            let mut b = 0xff;
            spi.transfer(slice::from_mut(&mut b))?;
            if b as usize > RESPONSES {
                return Err(Error::BufferTooSmall {
                    was: RESPONSES,
                    needed: b as usize,
                });
            }
            b as usize
        } else {
            Self::check_spi_byte(&mut spi, RESPONSES as u8)?;
            RESPONSES
        };

        let mut ret = heapless::Vec::new();

        for (i, response) in array::IntoIter::new(responses).enumerate() {
            if i == response_num {
                break;
            }

            let len = match config.param_len {
                ParamLen::U8 => {
                    let mut b = 0xff;
                    spi.transfer(slice::from_mut(&mut b))?;
                    b as usize
                }
                ParamLen::U16 => {
                    let mut x = [0xff; 2];
                    spi.transfer(&mut x)?;
                    u16::from_le_bytes(x) as usize
                }
            };

            if len > response.len() {
                return Err(Error::BufferTooSmall {
                    was: len,
                    needed: response.len(),
                });
            }

            spi.transfer(&mut response[..len])?;
            unsafe {
                ret.push_unchecked(&response[..len]);
            }
        }

        Self::check_spi_byte(&mut spi, END_CMD)?;

        Ok(ret)
    }

    fn send_cmd_and_receive<'a, const PARAMS: usize, const RESPONSES: usize>(
        &mut self,
        cmd: Cmd,
        params: [&[u8]; PARAMS],
        responses: [&'a mut [u8]; RESPONSES],
        config: SendReceiveConfig,
    ) -> Result<heapless::Vec<&'a [u8], RESPONSES>, Error<E>> {
        self.send_cmd(cmd, params, config.send)?;
        self.receive_cmd(cmd, responses, config.receive)
    }
}

/// The WiFi connection status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionStatus {
    Idle,
    NoSSIDAvailable,
    ScanCompleted,
    Connected,
    ConnectFailed,
    ConnectionLost,
    Disconnected,
    APListening,
    APConnected,
    APFailed,
}

/// Interface to the Adafruit Airlift WiFi peripheral.
///
/// This is currently a blocking interface, but will be async instead when
/// that's feasible.
pub struct AirliftWifi<SPI, CS, BUSY, RESET, GPIO0, TIMER> {
    driver: Driver<SPI, CS, BUSY, RESET, GPIO0, TIMER>,
}

impl<SPI, CS, BUSY, RESET, GPIO0, TIMER, E> AirliftWifi<SPI, CS, BUSY, RESET, GPIO0, TIMER>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    CS: OutputPin<Error = Infallible>,
    BUSY: InputPin<Error = Infallible>,
    RESET: OutputPin<Error = Infallible>,
    GPIO0: OutputPin<Error = Infallible>,
    TIMER: DelayMs<u32>,
{
    pub fn new(
        spi: SPI,
        cs: CS,
        busy: BUSY,
        reset: RESET,
        gpio0: GPIO0,
        timer: TIMER,
    ) -> Result<Self, E> {
        let mut driver = Driver {
            spi,
            cs,
            busy,
            reset,
            gpio0,
            timer,
        };
        driver.reset()?;

        Ok(Self { driver })
    }

    /// Retrieve the firmware version. Just guessing at the maximum length here.
    pub fn get_firmware_version(&mut self) -> Result<heapless::String<10>, Error<E>> {
        let mut b = [0; 10];

        let resp = self.driver.send_cmd_and_receive(
            Cmd::GetFirmwareVersion,
            [],
            [&mut b],
            SendReceiveConfig::default(),
        )?;

        match str::from_utf8(resp[0]) {
            Ok(s) => Ok(s.into()),
            Err(_) => Err(Error::InvalidEncoding),
        }
    }

    pub fn get_mac_address(&mut self) -> Result<[u8; 6], Error<E>> {
        let mut b = [0; 6];

        self.driver.send_cmd_and_receive(
            Cmd::GetMACAddress,
            [&[0xff]], // dummy data
            [&mut b],
            SendReceiveConfig::default(),
        )?;

        Ok(b)
    }

    pub fn get_status(&mut self) -> Result<ConnectionStatus, Error<E>> {
        let mut resp = 0;
        self.driver.send_cmd_and_receive(
            Cmd::GetConnStatus,
            [],
            [slice::from_mut(&mut resp)],
            SendReceiveConfig::default(),
        )?;

        Ok(match resp {
            0 => ConnectionStatus::Idle,
            1 => ConnectionStatus::NoSSIDAvailable,
            2 => ConnectionStatus::ScanCompleted,
            3 => ConnectionStatus::Connected,
            4 => ConnectionStatus::ConnectFailed,
            5 => ConnectionStatus::ConnectionLost,
            6 => ConnectionStatus::Disconnected,
            7 => ConnectionStatus::APListening,
            8 => ConnectionStatus::APConnected,
            9 => ConnectionStatus::APFailed,
            _ => return Err(Error::CmdFailed(Cmd::GetConnStatus)),
        })
    }

    pub fn scan_networks<const N: usize>(&mut self) -> Result<heapless::Vec<heapless::Vec<u8, 32>, N>, Error<E>> {
        let mut resp = 0;
        self.driver.send_cmd_and_receive(
            Cmd::StartScanNetworks,
            [],
            [slice::from_mut(&mut resp)],
            SendReceiveConfig::default()
        )?;

        if resp != 1 {
            return Err(Error::CmdFailed(Cmd::StartScanNetworks));
        }

        let mut bytes = [[0; 32]; N];
        let mut data = heapless::Vec::<&mut [u8], N>::new();
        for bytes in &mut bytes {
            unsafe {
                data.push_unchecked(bytes);
            }
        }

        let names = self.driver.send_cmd_and_receive(
            Cmd::ScanNetwork,
            [],
            data.into_array::<N>().unwrap(),
            SendReceiveConfig {
                receive: ReceiveConfig {
                    allow_fewer_resp: true,
                    ..Default::default()
                },
                ..Default::default()
            },
        )?;

        let mut v = heapless::Vec::new();

        for name in names {
            unsafe {
                v.push_unchecked(heapless::Vec::from_slice(name).unwrap());
            }
        }

        Ok(v)
    }

    pub fn connect_wifi(&mut self, ssid: &[u8], passphrase: Option<&[u8]>) -> Result<(), Error<E>> {
        if let Some(passphrase) = passphrase {
            let mut resp = 0;
            self.driver.send_cmd_and_receive(
                Cmd::SetPassPhrase,
                [ssid, passphrase],
                [slice::from_mut(&mut resp)],
                SendReceiveConfig::default(),
            )?;

            if resp != 1 {
                return Err(Error::CmdFailed(Cmd::SetPassPhrase));
            }
        } else {
            let mut resp = 0;
            self.driver.send_cmd_and_receive(
                Cmd::SetNet,
                [ssid],
                [slice::from_mut(&mut resp)],
                SendReceiveConfig::default(),
            )?;

            if resp != 1 {
                return Err(Error::CmdFailed(Cmd::SetNet));
            }
        }

        let mut count = 0;
        let status = loop {
            let status = self.get_status()?;

            if status == ConnectionStatus::Connected {
                return Ok(());
            }

            if count == 10 {
                break status;
            }

            count += 1;
            self.driver.timer.delay_ms(1000);
        };

        match status {
            ConnectionStatus::NoSSIDAvailable => Err(Error::NoSuchSSID),
            _ => Err(Error::ConnectionFailed),
        }
    }
}
