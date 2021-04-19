use crate::ccm;
use crate::iomuxc::consts::{Unsigned, U0, U1, U2, U3, U4};
use crate::iomuxc::sai;
use crate::ral;
use core::marker::PhantomData;

#[derive(Debug)]
pub struct Version {
    pub major: u8,
    pub minor: u8,
    pub feature: u16,
    pub frame_size: u8,
    pub fifo_size: u8,
    pub datalines: u8,
}

#[derive(PartialEq, Debug)]
pub enum BitClockSource {
    External,
    BusClock,
    Mclk1,
    Mclk2,
    Mclk3,
}

impl BitClockSource {
    fn msel(&self) -> u32 {
        match self {
            BitClockSource::External => 0,
            BitClockSource::BusClock => 0,
            BitClockSource::Mclk1 => 1,
            BitClockSource::Mclk2 => 2,
            BitClockSource::Mclk3 => 3,
        }
    }
}

pub struct BitClockConfig {
    /// Valid values are even numbers between 2 and 512, inclusive
    ///
    /// Specifying an odd number (or any number outside the range) will panic.
    pub divider: u32,

    pub active_high: bool,

    pub source: BitClockSource,
    // TODO: Add support for SYNC, BCS, and BCI
}

pub struct FifoStatus {
    pub error: bool,
    pub watermark: bool,
    pub empty: bool,
    pub full: bool,
}

pub struct SAI<M> {
    _module: PhantomData<M>,
    reg: ral::sai::Instance,
}

impl<M> SAI<M> {
    fn new(reg: ral::sai::Instance) -> Self {
        SAI {
            _module: PhantomData,
            reg,
        }
    }

    pub fn regs(&self) -> &ral::sai::Instance {
        &self.reg
    }

    /// Returns the version & param values of the SAI block
    pub fn version(&self) -> Version {
        Version {
            major: ral::read_reg!(ral::sai, self.reg, VERID, MAJOR) as u8,
            minor: ral::read_reg!(ral::sai, self.reg, VERID, MINOR) as u8,
            feature: ral::read_reg!(ral::sai, self.reg, VERID, FEATURE) as u16,
            frame_size: ral::read_reg!(ral::sai, self.reg, PARAM, FRAME) as u8,
            fifo_size: ral::read_reg!(ral::sai, self.reg, PARAM, FIFO) as u8,
            datalines: ral::read_reg!(ral::sai, self.reg, PARAM, DATALINE) as u8,
        }
    }

    /// Configure the clocks
    ///
    /// The divider can have even values between 2 and 512, inclusive. Other values will panic.
    ///
    /// Note: The SAI peripheral must not be running during this operation
    ///
    /// TODO: Add support for SYNC, BCS, BCI
    pub fn configure_bit_clock(&mut self, divider: u32, active_high: bool, source: BitClockSource) {
        if divider < 2 || divider > 512 || divider % 2 != 0 {
            panic!("Received invalid bit clock divider {}", divider);
        }
        let is_internal = if source != BitClockSource::External {
            1
        } else {
            0
        };
        ral::modify_reg!(ral::sai, self.reg, TCR2,
            DIV: divider / 2 - 1,
            BCD: is_internal,
            BCP: if active_high { 0 } else { 1 },
            MSEL: source.msel(),
            BCI: 0,
            BCS: 0,
            SYNC: 0
        );
    }

    /// TODO: Store the channel information in the SAI struct
    pub fn set_enabled_channels(&mut self, channels: &[u8]) {
        let mut channel_mask = 0;
        for channel in channels.iter() {
            channel_mask |= (1 << channel);
        }
        ral::modify_reg!(ral::sai, self.reg, TCR3, TCE: channel_mask);
    }

    /// `words` must be no greater than 32
    pub fn configure_frame(
        &mut self,
        words: u32,
        first_word_width: u32,
        word_width: u32,
        msb_first: bool,
    ) {
        assert!(words <= 32 && words > 0);
        ral::modify_reg!(ral::sai, self.reg, TCR4, FRSZ: words - 1, MF: if msb_first { 1 } else { 0 });
        ral::modify_reg!(
            ral::sai,
            self.reg,
            TCR5,
            W0W: first_word_width - 1,
            WNW: word_width - 1,
            // We don't support shifting the words within the FIFO, yet
            FBT: if msb_first { word_width - 1 } else { 0 }
        );
    }

    /// `width` must be between 1 and the length of the first word
    pub fn configure_frame_sync(
        &mut self,
        external: bool,
        active_high: bool,
        stop_on_warning: bool,
        early: bool,
        width: u32,
    ) {
        ral::modify_reg!(ral::sai, self.reg, TCR4,
            FSD: if external { 0 } else { 1 },
            FSP: if active_high { 0 } else { 1 },
            ONDEM: if stop_on_warning { 1 } else { 0 },
            FSE: if early { 1 } else { 0 },
            SYWD: width - 1
        );
    }

    pub fn fifo_status(&mut self) -> FifoStatus {
        let fifo_error = if ral::read_reg!(ral::sai, self.reg, TCSR, FEF) != 0 {
            true
        } else {
            false
        };
        //
        let tfr0 = ral::read_reg!(ral::sai, self.reg, TFR0);
        let wfp = (tfr0 >> 16) & 0b11_1111;
        let rfp = tfr0 & 0b11_1111;
        let is_full =
            ((wfp & 0b1_1111) == (rfp & 0b1_1111) && (wfp & 0b10_0000) != (rfp & 0b10_0000));
        let is_empty = wfp == rfp;
        FifoStatus {
            error: fifo_error,
            watermark: false,
            empty: is_empty,
            full: is_full,
        }
    }

    pub fn clear_fifo_error(&mut self) {
        ral::modify_reg!(ral::sai, self.reg, TCSR, FEF: 1);
    }

    pub fn push_sample(&mut self, sample: u32) {
        ral::write_reg!(ral::sai, self.reg, TDR0, TDR: sample);
    }

    pub fn set_tx_enable(&mut self, enable: bool) {
        ral::write_reg!(ral::sai, self.reg, TCSR,
            TE: if enable { 1 } else { 0 }
        );
    }
}

pub trait SaiClockingValues {
    /// The divider is split across a 3-bit and a 6-bit divider, so ranges from
    /// x1 to x512
    fn clock(ccm_handle: &mut ccm::Handle, clock_source: ccm::sai::ClockSource, divider: u32);
}

impl SaiClockingValues for U1 {
    fn clock(ccm_handle: &mut ccm::Handle, clock_source: ccm::sai::ClockSource, divider: u32) {
        match clock_source {
            ccm::sai::ClockSource::Pll4(_) => {
                ral::modify_reg!(ral::ccm, ccm_handle.base, CSCMR1,
                    SAI1_CLK_SEL: 0b10 // PLL4
                );
            }
        }

        // Pick the maximum predivider value that evenly divides the divider
        let mut prediv = 8;
        while divider % prediv != 0 {
            prediv -= 1;
        }
        let postdiv = divider / prediv;

        ral::write_reg!(
            ral::ccm,
            ccm_handle.base,
            CS1CDR,
            SAI1_CLK_PRED: prediv - 1,
            SAI1_CLK_PODF: postdiv - 1
        );

        ral::modify_reg!(ral::ccm, ccm_handle.base, CCGR5, CG9: 0b11);
    }
}

pub struct Unclocked<M> {
    pub(crate) module: PhantomData<M>,
    pub(crate) sai: ral::sai::Instance,
}

impl<M: SaiClockingValues + Unsigned> Unclocked<M> {
    pub fn clock(
        self,
        ccm_handle: &mut ccm::Handle,
        clock_source: ccm::sai::ClockSource,
        divider: u32,
    ) -> Builder<M> {
        M::clock(ccm_handle, clock_source, divider);

        Builder::new(self.sai)
    }
}

/// An SAI builder that can build a SAI peripheral
pub struct Builder<M> {
    _module: PhantomData<M>,
    reg: ral::sai::Instance,
}

impl<M> Builder<M>
where
    M: Unsigned,
{
    fn new(reg: ral::sai::Instance) -> Self {
        Builder {
            _module: PhantomData,
            reg,
        }
    }

    pub fn build_1bit_tx<MCLK, BCLK, SYNC, DATA, TXDATA>(
        self,
        mut mclk: MCLK,
        mut bclk: BCLK,
        mut sync: SYNC,
        mut data: DATA,
    ) -> SAI<M>
    where
        MCLK: sai::Pin<M, Signal = sai::Mclk>,
        BCLK: sai::Pin<M, Signal = sai::TxBclk>,
        SYNC: sai::Pin<M, Signal = sai::TxSync>,
        DATA: sai::Pin<M, Signal = TXDATA>,
        TXDATA: sai::TxDataSignal<Index = U0>,
    {
        crate::iomuxc::sai::prepare(&mut mclk);
        crate::iomuxc::sai::prepare(&mut bclk);
        crate::iomuxc::sai::prepare(&mut sync);
        crate::iomuxc::sai::prepare(&mut data);

        SAI::new(self.reg)
    }
}
