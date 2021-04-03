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

pub enum Mclk {
    External,
    BusClock,
    Mclk1,
    Mclk2,
    Mclk3,
}

impl Mclk {
    fn msel(&self) -> u32 {
        match self {
            External => 0,
            BusClock => 0,
            Mclk1 => 1,
            Mclk2 => 2,
            Mclk3 => 3,
        }
    }
}

pub struct ClockConfig {
    pub mclk: Mclk,
    pub bclk_external: bool,
    pub bclk_active_high: bool,
    pub bclk_div: u8,
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
}

pub struct Unclocked {
    pub(crate) sai1: ral::sai::Instance,
    pub(crate) sai2: ral::sai::Instance,
    pub(crate) sai3: ral::sai::Instance,
}

impl Unclocked {
    pub fn clock(self, ccm_handle: &mut ccm::Handle) -> (Builder<U1>, Builder<U2>, Builder<U3>) {
        let (ccm, _) = ccm_handle.raw();
        //ral::modify_reg!(ral::ccm, ccm, CCGR1, CG8: 0b11); // adc1_clk_enable
        //ral::modify_reg!(ral::ccm, ccm, CCGR1, CG4: 0b11); // adc2_clk_enable
        ral::modify_reg!(ral::ccm, ccm, CCGR5, CG9: 0b11, CG10: 0b11, CG11: 0b11);

        (
            Builder::new(self.sai1),
            Builder::new(self.sai2),
            Builder::new(self.sai3),
        )
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

    pub fn build_1bit_tx<MCLK, BCLK, SYNC, DATA>(
        self,
        clock_config: ClockConfig,
        mut mclk: MCLK,
        mut bclk: BCLK,
        mut sync: SYNC,
        mut data: DATA,
    ) -> SAI<M>
    where
        MCLK: sai::MclkPin<Module = M>,
        BCLK: sai::BclkPin<Module = M>,
        SYNC: sai::SyncPin<Module = M>,
        DATA: sai::TxDataPin<Module = M, Index = U0>,
    {
        crate::iomuxc::sai::prepare_mclk(&mut mclk);
        crate::iomuxc::sai::prepare_bclk(&mut bclk);
        crate::iomuxc::sai::prepare_sync(&mut sync);
        crate::iomuxc::sai::prepare_txdata(&mut data);

        /*
        pub enum Mclk {
            External,
            BusClock,
            Mclk1,
            Mclk2,
            Mclk3,
        }

        pub struct ClockConfig {
            pub mclk: Mclk,
            pub bclk_external: bool,
            pub bclk_active_high: bool,
            pub bclk_div: u8,
        }
                */
        //ral::write_reg!(ral::sai, self.reg, TCR2, DIV: clock_config.bclk_div as u32, BCD: !clock_config.bclk_external as u32, BCP: !clock_config.bclk_active_high as u32, MSEL: clock_config.mclk.msel(), BCI: 0, BCS: 0, SYNC: 0);
        /*ral::write_reg!(ral::sai, self.reg, TCR5,
        FBT: 0, // Not shifted
        W0W: ..., WNW: ...);*/

        SAI::new(self.reg)
    }

    /*/// Builds an I2C peripheral from the SCL and SDA pins. The return
    /// is a configured I2C master running at 100KHz.
    pub fn build<SCL, SDA>(self, mut scl: SCL, mut sda: SDA) -> I2C<M>
    where
        SCL: i2c::Pin<Module = M, Signal = i2c::SCL>,
        SDA: i2c::Pin<Module = M, Signal = i2c::SDA>,
    {
        crate::iomuxc::i2c::prepare(&mut scl);
        crate::iomuxc::i2c::prepare(&mut sda);
        I2C::new(self.source_clock, self.reg)
    }*/
}
