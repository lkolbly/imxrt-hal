//! ADC
//!
//! This ADC driver supports `embedded_hal`'s ADC traits
//!
//! # Example
//! ```no_run
//! use imxrt_hal::{self, adc};
//! use embedded_hal::adc::OneShot;
//!
//! let mut peripherals = imxrt_hal::Peripherals::take().unwrap();
//! let (adc1_builder, _) = peripherals.adc.clock(&mut peripherals.ccm.handle);
//!
//! let mut adc1 = adc1_builder.build(adc::ClockSelect::default(), adc::ClockDivision::default());
//! let mut a1 = adc::AnalogInput::new(peripherals.iomuxc.ad_b1.p02);
//!
//! let reading: u16 = adc1.read(&mut a1).unwrap();
//!```
//!
//! The ADC starts out with a default configuration of 4 hardware samples, a conversion speed of
//! medium, a resolution of 10 bits, and low power mode disabled. It's also pre-calibrated using
//! 32 averages and a slow conversion speed.

use crate::ccm;
use crate::iomuxc::adc::{prepare, Pin, ADC1, ADC2};
use crate::iomuxc::{adc, consts::Unsigned};
use crate::ral;
use core::marker::PhantomData;
use embedded_hal::adc::{Channel, OneShot};

/// The clock input for an ADC
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockSelect {
    IPG(super::ccm::IPGFrequency),   // IPG clock
    IPG_2(super::ccm::IPGFrequency), // IPG clock / 2
    ADACK,                           // Asynchronous clock
}

/// How much to divide the clock input
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockDivision {
    Div1, // Input clock / 1
    Div2, // Input clock / 2
    Div4, // Input clock / 4
    Div8, // Input clock / 8
}

impl ClockSelect {
    pub fn default() -> Self {
        ClockSelect::ADACK
    }
}

impl ClockDivision {
    pub fn default() -> Self {
        ClockDivision::Div2
    }
}

/// Conversion speeds done by clock cycles
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConversionSpeed {
    Slow,     // 25 ADC clock cycles (24 on imxrt102x)
    Medium,   // 17 ADC clock cycles (16 on imxrt102x)
    Fast,     // 9 ADC clock cycles (8 on imxrt102x)
    VeryFast, // 3 ADC clock cycles (2 on imxrt102x)
}

/// Denotes how much hardware averaging to do
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AveragingCount {
    Avg1,
    Avg4,
    Avg8,
    Avg16,
    Avg32,
}

// Specifies the resolution the ADC
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResolutionBits {
    Res8,
    Res10,
    Res12,
}

/// A pin representing an analog input for a particular ADC
pub struct AnalogInput<ADCx, P> {
    _module: PhantomData<ADCx>,
    pin: P,
}

impl<P, ADCx> Channel<ADCx> for AnalogInput<ADCx, P>
where
    P: Pin<ADCx>,
    ADCx: adc::ADC,
{
    type ID = u16;

    fn channel() -> Self::ID {
        <P as Pin<ADCx>>::Input::U16
    }
}

impl<P, ADCx> AnalogInput<ADCx, P>
where
    P: Pin<ADCx>,
    ADCx: adc::ADC,
{
    /// Creates a new analog input pin
    pub fn new(mut pin: P) -> Self {
        prepare(&mut pin);
        Self {
            _module: PhantomData,
            pin,
        }
    }

    /// Release the ADC input, returning the underlying hardware pin. This pin is in an
    /// unspecified state
    pub fn release(self) -> P {
        self.pin
    }
}

pub struct ADC<ADCx> {
    _module: PhantomData<ADCx>,
    reg: ral::adc::Instance,
}

impl<ADCx> ADC<ADCx> {
    fn new(reg: ral::adc::Instance) -> Self {
        let mut inst = Self {
            _module: PhantomData,
            reg,
        };

        inst.set_resolution(ResolutionBits::Res10);
        inst.set_low_power_mode(false);

        // Calibrate w/ slow settings initially
        inst.set_averaging(AveragingCount::Avg32);
        inst.set_conversion_speed(ConversionSpeed::Slow);
        inst.calibrate();

        // Set to default of 4 hardware averages & medium conversion speed
        inst.set_averaging(AveragingCount::Avg4);
        inst.set_conversion_speed(ConversionSpeed::Medium);

        inst
    }

    pub fn get_regs(&mut self) -> &ral::adc::Instance {
        &self.reg
    }

    /// Sets the resolution that analog reads return, in bits
    pub fn set_resolution(&mut self, bits: ResolutionBits) {
        ral::modify_reg!(ral::adc, self.reg, CFG, MODE: match bits {
            ResolutionBits::Res8 => MODE_0,
            ResolutionBits::Res10 => MODE_1,
            ResolutionBits::Res12 => MODE_2
        });
    }

    /// Sets the number of hardware averages taken by the ADC
    pub fn set_averaging(&mut self, avg: AveragingCount) {
        ral::modify_reg!(ral::adc, self.reg, GC, AVGE: match avg {
            AveragingCount::Avg1 => AVGE_0,
            _ => AVGE_1
        });
        ral::modify_reg!(ral::adc, self.reg, CFG, AVGS: match avg {
            AveragingCount::Avg32 => AVGS_3,
            AveragingCount::Avg16 => AVGS_2,
            AveragingCount::Avg8 => AVGS_1,
            _ => AVGS_0,
        });
    }

    /// Sets the conversion speed for this ADC, see ConversionSpeed for clock cycle counts.
    pub fn set_conversion_speed(&mut self, conversion_speed: ConversionSpeed) {
        ral::modify_reg!(ral::adc, self.reg, CFG,
            ADSTS: match conversion_speed {
                ConversionSpeed::Slow => ADSTS_3,
                ConversionSpeed::Medium => ADSTS_1,
                ConversionSpeed::Fast => ADSTS_3,
                ConversionSpeed::VeryFast => ADSTS_0
            },
            ADLSMP: match conversion_speed {
                ConversionSpeed::Slow => ADLSMP_1,
                ConversionSpeed::Medium => ADLSMP_1,
                ConversionSpeed::Fast => ADLSMP_0,
                ConversionSpeed::VeryFast => ADLSMP_0
            }
        );
    }

    /// Enables or disables the low power configuration in the ADC. This does limit the
    /// ADACK clock frequency (<= 20MHz)
    pub fn set_low_power_mode(&mut self, state: bool) {
        ral::modify_reg!(ral::adc, self.reg, CFG, ADLPC: if state { ADLPC_1 } else { ADLPC_0 });
    }

    /// Calibrates the ADC, will wait for finish
    pub fn calibrate(&mut self) {
        ral::modify_reg!(ral::adc, self.reg, GC, CAL: 0b1);
        while (ral::read_reg!(ral::adc, self.reg, CAL, CAL_CODE) != 0) {}
    }
}

/// Implement embedded-hal traits
impl<ADCx, WORD, P> OneShot<ADCx, WORD, AnalogInput<ADCx, P>> for ADC<ADCx>
where
    ADCx: adc::ADC,
    WORD: From<u16>,
    P: Pin<ADCx>,
{
    type Error = core::convert::Infallible;

    /// Read an ADC value from an AnalogInput. This should always return a good result
    fn read(&mut self, _pin: &mut AnalogInput<ADCx, P>) -> nb::Result<WORD, Self::Error> {
        let channel = <P as Pin<ADCx>>::Input::U32;
        ral::modify_reg!(ral::adc, self.reg, HC0, |_| channel);
        while (ral::read_reg!(ral::adc, self.reg, HS, COCO0) == 0) {}

        Ok((ral::read_reg!(ral::adc, self.reg, R0) as u16).into())
    }
}

/// Internal trait for determining what DMA request signal to use for
/// ADC1 or ADC2.
#[doc(hidden)]
pub trait AdcDmaSource {
    /// See table 4-3 of the iMXRT1060 Reference Manual (Rev 2)
    const SOURCE_REQUEST_SIGNAL: u32;
}

impl AdcDmaSource for ADC1 {
    const SOURCE_REQUEST_SIGNAL: u32 = 24;
}

impl AdcDmaSource for ADC2 {
    const SOURCE_REQUEST_SIGNAL: u32 = 88;
}

/// Streaming DMA source for ADCs
pub struct AdcSource<ADCx, P> {
    adc: ADC<ADCx>,
    #[allow(unused)]
    // Placeholder for a future implementation; remove this allow when using the pin!
    pin: AnalogInput<ADCx, P>,
}

impl<ADCx, P> AdcSource<ADCx, P>
where
    ADCx: adc::ADC + AdcDmaSource,
    P: Pin<ADCx>,
{
    /// Create a DMA Source object tieing the ADC to the given pin.
    pub fn new(adc: ADC<ADCx>, pin: AnalogInput<ADCx, P>) -> Self {
        AdcSource { adc, pin }
    }
}

impl<ADCx, P> crate::dma::peripheral::Source<u16> for AdcSource<ADCx, P>
where
    ADCx: adc::ADC + AdcDmaSource,
    P: Pin<ADCx>,
{
    type Error = ();

    const SOURCE_REQUEST_SIGNAL: u32 = ADCx::SOURCE_REQUEST_SIGNAL;

    fn source(&self) -> *const u16 {
        &self.adc.reg.R0 as *const _ as *const u16
    }

    fn enable_source(&mut self) -> Result<(), Self::Error> {
        let channel = <P as Pin<ADCx>>::Input::U32;
        ral::modify_reg!(ral::adc, self.adc.reg, GC, ADCO: 1, DMAEN: 1);
        ral::modify_reg!(ral::adc, self.adc.reg, HC0, |_| channel);
        Ok(())
    }

    fn disable_source(&mut self) {
        ral::modify_reg!(ral::adc, self.adc.reg, GC, ADCO: 0, DMAEN: 0);
    }
}

/// StreamingAdc wraps AdcSource and the DMA machinery to provide a nice
/// user interface for reading regular samples from the ADC.
///
/// This implementation uses two linear buffers and switches between them,
/// yielding 256 samples at a time.
///
/// # Example
///
/// ```no_run
/// /*static ADC_BUFFER: hal::dma::Buffer<[u16; 256]> = hal::dma::Buffer::new([0; 256]);
/// static ADC_BUFFER2: hal::dma::Buffer<[u16; 256]> = hal::dma::Buffer::new([0; 256]);
/// static mut streaming_adc: Option<hal::adc::StreamingAdc<hal::iomuxc::adc::ADC1, t41::P24>> = None;
///
/// #[interrupt]
/// fn DMA8_DMA24() {
///     unsafe {
///         if let Some(adc) = &mut streaming_adc {
///             adc.handle_interrupt();
///         }
///     }
/// }
///
/// ...
///
/// let (adc1_builder, _) = adc.clock(&mut ccm.handle);
/// let mut adc1 = adc1_builder.build(
///     adc::ClockSelect::default(),
///     adc::ClockDivision::default()
/// );
/// let mut a1 = adc::AnalogInput::new(pins.p24);
///
/// let mut channel = dma_channels[8].take().unwrap();
/// channel.set_interrupt_on_completion(true);
/// unsafe {
///     streaming_adc = Some(hal::adc::StreamingAdc::new(
///         adc1,
///         a1,
///         channel,
///         &ADC_BUFFER,
///         &ADC_BUFFER2
///     ));
///     cortex_m::peripheral::NVIC::unmask(interrupt::DMA8_DMA24);
/// }
///
/// ...
///
/// // Inside the main loop
/// let mut sadc = unsafe { streaming_adc.as_mut().unwrap() };
/// sadc.poll(|buf| {
///     adc_min = *buf.iter().min().unwrap();
///     adc_max = *buf.iter().max().unwrap();
/// });*/
/// ```
pub struct StreamingAdc<ADCx, P> {
    buffer0: &'static crate::dma::Buffer<[u16; 256]>,
    buffer1: &'static crate::dma::Buffer<[u16; 256]>,
    peripheral: crate::dma::Peripheral<
        AdcSource<ADCx, P>,
        u16,
        crate::dma::Linear<u16>,
        crate::dma::Linear<u16>,
    >,
    unhandled_buffer_count: core::sync::atomic::AtomicUsize,
    off_buffer: crate::dma::Linear<u16>,
    _pin: PhantomData<P>,
}

impl<ADCx, P> StreamingAdc<ADCx, P>
where
    ADCx: adc::ADC + AdcDmaSource,
    P: Pin<ADCx>,
{
    /// Create a new StreamingAdc object. The returned object will not begin
    /// generating samples until start() is called.
    pub fn new(
        adc: ADC<ADCx>,
        pin: AnalogInput<ADCx, P>,
        channel: crate::dma::Channel,
        buffer0: &'static crate::dma::Buffer<[u16; 256]>,
        buffer1: &'static crate::dma::Buffer<[u16; 256]>,
    ) -> Self {
        let mut off_buffer = crate::dma::Linear::new(&buffer0).unwrap();
        off_buffer.set_transfer_len(256);
        let mut peripheral = crate::dma::receive_u16(AdcSource::new(adc, pin), channel);

        Self {
            buffer0,
            buffer1,
            peripheral,
            unhandled_buffer_count: core::sync::atomic::AtomicUsize::new(0),
            off_buffer,
            _pin: PhantomData,
        }
    }

    /// Begin transferring samples from the ADC. Once samples are flowing, the
    /// user must call poll() frequently enough to keep up with the generated
    /// samples, or else the interrupt handler will panic.
    pub fn start(&mut self) {
        let mut buffer = crate::dma::Linear::new(&self.buffer1).unwrap();
        buffer.set_transfer_len(256);
        self.peripheral.start_receive(buffer).unwrap();
    }

    /// Process any received samples from the ADC. If no samples have been
    /// received since the last poll() call, cb() will not be called.
    pub fn poll<F: FnOnce(&[u16])>(&mut self, cb: F) {
        if self
            .unhandled_buffer_count
            .load(core::sync::atomic::Ordering::SeqCst)
            > 0
        {
            cb(self.off_buffer.as_elements());
            self.unhandled_buffer_count
                .store(0, core::sync::atomic::Ordering::SeqCst);
        }
    }

    /// Called from an interrupt handler. The user must register the appropriate
    /// DMA handler for their channel and call this method from it.
    ///
    /// If poll() is not called frequently enough to keep up with the data, this
    /// handler will panic.
    pub fn handle_interrupt(&mut self) {
        if self.peripheral.is_receive_interrupt() {
            self.peripheral.receive_clear_interrupt();
        }

        let mut rx_buffer = self.peripheral.receive_complete().unwrap();
        core::mem::swap(&mut self.off_buffer, &mut rx_buffer);
        self.peripheral.start_receive(rx_buffer).unwrap();
        if self
            .unhandled_buffer_count
            .swap(1, core::sync::atomic::Ordering::SeqCst)
            > 0
        {
            panic!("User is not processing ADC data fast enough!");
        }
    }
}

/// Unclocked ADC modules
///
/// The `Unclocked` struct represents both unconfigured ADC peripherals.
/// Once clocked, you'll have the ability to use either the ADC1 or ADC2
/// peripherals.
pub struct Unclocked {
    pub(crate) adc1: ral::adc::Instance,
    pub(crate) adc2: ral::adc::Instance,
}

impl Unclocked {
    pub fn clock(self, ccm_handle: &mut ccm::Handle) -> (Builder<ADC1>, Builder<ADC2>) {
        let (ccm, _) = ccm_handle.raw();
        ral::modify_reg!(ral::ccm, ccm, CCGR1, CG8: 0b11); // adc1_clk_enable
        ral::modify_reg!(ral::ccm, ccm, CCGR1, CG4: 0b11); // adc2_clk_enable

        (Builder::new(self.adc1), Builder::new(self.adc2))
    }
}

/// An ADC builder than can build an ADC1 or ADC2 module
pub struct Builder<ADCx> {
    _module: PhantomData<ADCx>,
    reg: ral::adc::Instance,
}

impl<ADCx> Builder<ADCx>
where
    ADCx: adc::ADC,
{
    fn new(reg: ral::adc::Instance) -> Self {
        Self {
            _module: PhantomData,
            reg,
        }
    }

    /// Builds an ADC peripheral with a certain clock selection. The ADC starts at
    /// a 10-bit resolution w/ 4 hardware averaging samples, at medium speed, and
    /// with low power mode disabled.
    pub fn build(self, clock: ClockSelect, division: ClockDivision) -> ADC<ADCx> {
        // Enable asynchronous clock if applicable
        ral::modify_reg!(ral::adc, self.reg, GC, ADACKEN: match clock {
            ClockSelect::ADACK => ADACKEN_1,
            _ => ADACKEN_0
        });

        // Select the clock selection, division, and enable ADHSC if applicable
        ral::modify_reg!(ral::adc, self.reg, CFG,
            ADICLK: match clock {
                ClockSelect::IPG(_ipg_frequency) => ADICLK_0,
                ClockSelect::IPG_2(_ipg_frequency) => ADICLK_1,
                ClockSelect::ADACK => ADICLK_3
            },
            ADIV: match division {
                ClockDivision::Div1 => ADIV_0,
                ClockDivision::Div2 => ADIV_1,
                ClockDivision::Div4 => ADIV_2,
                ClockDivision::Div8 => ADIV_3
            },
            ADHSC: ADHSC_1
        );

        ADC::new(self.reg)
    }
}
