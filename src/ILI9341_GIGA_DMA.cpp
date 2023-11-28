/***************************************************
 * The Async(DMA) support functions.
 ***************************************************/

#include "ILI9341_GIGA_n.h"
#include <SPI.h>
#include <api/itoa.h>
#include "pinDefinitions.h"
#include <LibPrintf.h>
#include <GIGA_digitalWriteFast.h>

//#define DEBUG_ASYNC_UPDATE  // Enable to print out dma info
//#define DEBUG_ASYNC_LEDS	// Enable to use digitalWrites to Debug

#define WIDTH ILI9341_TFTWIDTH
#define HEIGHT ILI9341_TFTHEIGHT
#define CBALLOC (ILI9341_TFTHEIGHT * ILI9341_TFTWIDTH * 2)

ILI9341_GIGA_n *ILI9341_GIGA_n::_dmaActiveDisplay[2] = {0, 0};

// Forward references;
extern void dump_spi_settings(SPI_TypeDef *pgigaSpi);
extern void dump_dma_settings(const char *sz, DMA_TypeDef *pdma);
extern void dump_dma_stream_settings(DMA_Stream_TypeDef *dmas);
extern void dump_dmamux(DMAMUX_Channel_TypeDef *dmuxc);



//=======================================================================
//=======================================================================
bool ILI9341_GIGA_n::updateScreenAsync(bool update_cont) {
	if (!_use_fbtft) return false;
	if (update_cont) return false;

	if (_dma_state & ILI9341_DMA_ACTIVE) { return false; }


	initDMASettings();

	SCB_CleanInvalidateDCache_by_Addr( _pfbtft, CBALLOC);

	// reset the buffers.
	_dmaStream->M0AR = (uint32_t)_pfbtft;
	_dmaStream->NDTR = 38400;  // (32*240/2);  //0 - 65536

	// Lets setup the transfer ... everything before the fill screen.
	beginSPITransaction(_SPI_CLOCK);
	// Doing full window.
	setAddr(0, 0, width() - 1, height() - 1);
	writecommand_cont(ILI9341_RAMWR);
	setDataMode();

	setSPIDataSize(16);
	_dma_sub_frame_count = 0;

	// enable RX
	SET_BIT(_dmaStream->CR, DMA_SxCR_EN_Msk);

	// Enable TX
	SET_BIT(_pgigaSpi->CFG1, SPI_CFG1_TXDMAEN);  // enable SPI TX
	// finally enable SPI
	SET_BIT(_pgigaSpi->CR1, SPI_CR1_SPE);     // enable SPI
	SET_BIT(_pgigaSpi->CR1, SPI_CR1_CSTART);  // enable SPI

	_dma_state |= ILI9341_DMA_ACTIVE;
	dumpDMASettings();

	return true;
}

//=======================================================================
//=======================================================================
void ILI9341_GIGA_n::waitUpdateAsyncComplete(void) {
	while ((_dma_state & ILI9341_DMA_ACTIVE)) {
		// asm volatile("wfi");
	};

}

//=======================================================================
//=======================================================================
void ILI9341_GIGA_n::endUpdateAsync() {
	if (_dma_state & ILI9341_DMA_CONT) {
		_dma_state &= ~ILI9341_DMA_CONT; // Turn of the continueous mode
	}

}


//=======================================================================
//=======================================================================
void ILI9341_GIGA_n::initDMASettings(void) {
	if (_dma_state & ILI9341_DMA_INIT) return;

	// See which dmastream we are using.There is probably a cleaner way to do this...
	uint32_t denable = 0;
	switch ((uint32_t)_dmaStream) {
	default: return;  // not in our list.
	case DMA1_Stream0_BASE: _dmamux = DMAMUX1_Channel0; _dmaTXIrq = DMA1_Stream0_IRQn; denable = RCC_AHB1ENR_DMA1EN; break;
	case DMA1_Stream1_BASE: _dmamux = DMAMUX1_Channel1; _dmaTXIrq = DMA1_Stream1_IRQn; denable = RCC_AHB1ENR_DMA1EN; break;
	case DMA1_Stream2_BASE: _dmamux = DMAMUX1_Channel2; _dmaTXIrq = DMA1_Stream2_IRQn; denable = RCC_AHB1ENR_DMA1EN; break;
	case DMA1_Stream3_BASE: _dmamux = DMAMUX1_Channel3; _dmaTXIrq = DMA1_Stream3_IRQn; denable = RCC_AHB1ENR_DMA1EN; break;
	case DMA1_Stream4_BASE: _dmamux = DMAMUX1_Channel4; _dmaTXIrq = DMA1_Stream4_IRQn; denable = RCC_AHB1ENR_DMA1EN; break;
	case DMA1_Stream5_BASE: _dmamux = DMAMUX1_Channel5; _dmaTXIrq = DMA1_Stream5_IRQn; denable = RCC_AHB1ENR_DMA1EN; break;
	case DMA1_Stream6_BASE: _dmamux = DMAMUX1_Channel6; _dmaTXIrq = DMA1_Stream6_IRQn; denable = RCC_AHB1ENR_DMA1EN; break;
	case DMA1_Stream7_BASE: _dmamux = DMAMUX1_Channel7; _dmaTXIrq = DMA1_Stream7_IRQn; denable = RCC_AHB1ENR_DMA1EN; break;
	case DMA2_Stream0_BASE: _dmamux = DMAMUX2_Channel0; _dmaTXIrq = DMA2_Stream0_IRQn; denable = RCC_AHB1ENR_DMA2EN; break;
	case DMA2_Stream1_BASE: _dmamux = DMAMUX2_Channel1; _dmaTXIrq = DMA2_Stream1_IRQn; denable = RCC_AHB1ENR_DMA2EN; break;
	case DMA2_Stream2_BASE: _dmamux = DMAMUX2_Channel2; _dmaTXIrq = DMA2_Stream2_IRQn; denable = RCC_AHB1ENR_DMA2EN; break;
	case DMA2_Stream3_BASE: _dmamux = DMAMUX2_Channel3; _dmaTXIrq = DMA2_Stream3_IRQn; denable = RCC_AHB1ENR_DMA2EN; break;
	case DMA2_Stream4_BASE: _dmamux = DMAMUX2_Channel4; _dmaTXIrq = DMA2_Stream4_IRQn; denable = RCC_AHB1ENR_DMA2EN; break;
	case DMA2_Stream5_BASE: _dmamux = DMAMUX2_Channel5; _dmaTXIrq = DMA2_Stream5_IRQn; denable = RCC_AHB1ENR_DMA2EN; break;
	case DMA2_Stream6_BASE: _dmamux = DMAMUX2_Channel6; _dmaTXIrq = DMA2_Stream6_IRQn; denable = RCC_AHB1ENR_DMA2EN; break;
	case DMA2_Stream7_BASE: _dmamux = DMAMUX2_Channel7; _dmaTXIrq = DMA2_Stream7_IRQn; denable = RCC_AHB1ENR_DMA2EN; break;
	}

	_dma_state |= ILI9341_DMA_INIT | ILI9341_DMA_EVER_INIT;

	// Enable DMA1
	SET_BIT(RCC->AHB1ENR, denable);
	delay(1000);

	_dmaStream->M0AR = (uint32_t)_pfbtft;
	_dmaStream->PAR = (uint32_t)&_pgigaSpi->TXDR;


	uint32_t cr = 0;
	cr |= 1 << DMA_SxCR_DIR_Pos;      // Set Memory to Peripheral
	cr |= (2 << DMA_SxCR_MSIZE_Pos);  // try 32 set 16 bit mode
	cr |= DMA_SxCR_MINC;              // Memory Increment
	cr |= 1 << DMA_SxCR_PSIZE_Pos;    // Peripheral size 16 bits
	//cr |= DMA_SxCR_PFCTRL;            // Peripheral in control of flow contrl
	cr |= DMA_SxCR_TCIE;            // interrupt on completion
	cr |= DMA_SxCR_TEIE;            // Interrupt on error.
	cr |= (3u << DMA_SxCR_PL_Pos);  // Very high priority
	cr |= (0x1 << DMA_SxCR_MBURST_Pos); // Incr 4...
	_dmaStream->CR = cr;

	// Experiment with FIFO not direct
	_dmaStream->FCR |= (0x3 << DMA_SxFCR_FTH_Pos) | DMA_SxFCR_DMDIS;  // disable Direct mode
	_dmaStream->FCR |= DMA_SxFCR_FEIE;  // Enable interrupt on FIFO error

	/******************MEM Address to buffer**************/
	/*******************Number of data transfer***********/
	_dmaStream->NDTR = 38400;  // (32*240/2);  //0 - 65536

	/******************Periph request**********************/
	// Point to our SPI
	_dmaActiveDisplay[_spi_num] = this;
	_dmamux->CCR = (_dmamux->CCR & ~(DMAMUX_CxCR_DMAREQ_ID_Msk))
	               | (s_spi_hardware_mapping[_spi_num].tx_dmamux1_req_id << DMAMUX_CxCR_DMAREQ_ID_Pos);

	// transfer complete and error interupt
	DMA1->LIFCR = DMA_LIFCR_CTCIF1;  //Clear IT in LISR Register
	NVIC_SetVector(_dmaTXIrq, (uint32_t)s_spi_hardware_mapping[_spi_num].txdmaisr);
	NVIC_EnableIRQ(_dmaTXIrq);
}

//=======================================================================
//=======================================================================
void ILI9341_GIGA_n::dumpDMASettings() {
	if (!(_dma_state & ILI9341_DMA_INIT)) return;

	dump_spi_settings(_pgigaSpi);

	if ((_dmaStream >= DMA1_Stream0) && (_dmaStream <= DMA1_Stream7)) dump_dma_settings("DMA1", DMA1);
	else dump_dma_settings("DMA2", DMA2);

	dump_dma_stream_settings(_dmaStream);
	dump_dmamux(_dmamux);

}


//=======================================================================
//=======================================================================
void ILI9341_GIGA_n::abortUpdateAsync() {
	CLEAR_BIT(_pgigaSpi->CFG1, SPI_CFG1_RXDMAEN);  // disable SPI RX
	CLEAR_BIT(_pgigaSpi->CFG1, SPI_CFG1_TXDMAEN);  // disable SPI TX
	setSPIDataSize(8);  // restore back to 8 bits
}


volatile uint32_t txIRQCount = 0;
uint32_t M0AR_at_irq[2];

//=======================================================================
//=======================================================================
// TX handler
void ILI9341_GIGA_n::dmaInterrupt(void) {
	if (_dmaActiveDisplay[0]) {
		_dmaActiveDisplay[0]->process_dma_interrupt();
	}
}
void ILI9341_GIGA_n::dmaInterrupt1(void) {
	if (_dmaActiveDisplay[1]) {
		_dmaActiveDisplay[1]->process_dma_interrupt();
	}
}
void ILI9341_GIGA_n::process_dma_interrupt(void) {
	txIRQCount++;
	digitalToggleFast(LED_BLUE);
	if (DMA1->LISR & DMA_LISR_TCIF1) {
		DMA1->LIFCR = DMA_LIFCR_CTCIF1;
		if (_dma_sub_frame_count == 0) {
			_dma_sub_frame_count = 1;
			M0AR_at_irq[0] = (uint32_t)_dmaStream->M0AR;
			_dmaStream->M0AR = (uint32_t)(&_pfbtft[38400]);
			_dmaStream->NDTR = 38400;                      // (32*240/2);  //0 - 65536
			SET_BIT(_pgigaSpi->CFG1, SPI_CFG1_RXDMAEN);  // enable SPI RX

			// enable the two streams
			SET_BIT(_dmaStream->CR, DMA_SxCR_EN_Msk);
			SET_BIT(_pgigaSpi->CR1, SPI_CR1_SPE);  // enable SPI


			// Enable TX
			SET_BIT(_pgigaSpi->CFG1, SPI_CFG1_TXDMAEN);  // enable SPI TX
		} else if (_dma_sub_frame_count == 1) {
			M0AR_at_irq[1] = (uint32_t)_dmaStream->M0AR;
			_dma_state &= ~ILI9341_DMA_ACTIVE;
			endSPITransaction();

			CLEAR_BIT(_pgigaSpi->CR1, SPI_CR1_SPE);        // disable SPI
			CLEAR_BIT(_pgigaSpi->CFG1, SPI_CFG1_RXDMAEN);  // disable SPI RX
			CLEAR_BIT(_pgigaSpi->CFG1, SPI_CFG1_TXDMAEN);  // disable SPI TX

			_pgigaSpi->CFG1 = (_pgigaSpi->CFG1 & ~(SPI_CFG1_DSIZE_Msk | SPI_CFG1_CRCSIZE_Msk))
			                  | (7 << SPI_CFG1_DSIZE_Pos) | (7 << SPI_CFG1_CRCSIZE_Pos);
			_pgigaSpi->CFG1 = (_pgigaSpi->CFG1 & ~(SPI_CFG1_FTHLV_Msk))
			                  | (7 << SPI_CFG1_FTHLV_Pos);
			SET_BIT(_pgigaSpi->CR1, SPI_CR1_SPE);  // enable SPI

		} else {
			// ??? error
		}
	}
	if (DMA1->LISR & DMA_LISR_TEIF1) {
		digitalToggleFast(LED_RED);
		DMA1->LIFCR = DMA_LIFCR_CTEIF1;
	}
	if (DMA1->LISR & DMA_LISR_FEIF1) {
		digitalToggleFast(LED_RED);
		DMA1->LIFCR = DMA_LIFCR_CFEIF1;
	}
}


//=======================================================================
//=======================================================================
void  ILI9341_GIGA_n::setSPIDataSize(uint8_t datasize) {

	datasize--;  // decrement by 1
	// lets disable SPI and set to 16 bit mode
	CLEAR_BIT(_pgigaSpi->CR1, SPI_CR1_SPE);  // disable SPI
	_pgigaSpi->CFG1 = (_pgigaSpi->CFG1 & ~(SPI_CFG1_DSIZE_Msk | SPI_CFG1_CRCSIZE_Msk))
	                  | (datasize << SPI_CFG1_DSIZE_Pos) | (datasize << SPI_CFG1_CRCSIZE_Pos);
	_pgigaSpi->CFG1 = (_pgigaSpi->CFG1 & ~(SPI_CFG1_FTHLV_Msk))
	                  | (1 << SPI_CFG1_FTHLV_Pos);

	SET_BIT(_pgigaSpi->CR1, SPI_CR1_SPE);  // enable SPI
}


//=======================================================================
// Print SPI Setting
//=======================================================================
void dump_spi_settings(SPI_TypeDef *pgigaSpi) {
	Serial.println("\n**SPI Settings**");
	uint32_t reg = pgigaSpi->CR1;
	Serial.print("  CR1:  ");
	Serial.println(reg, HEX); /*!< SPI/I2S Control register 1,                      Address offset: 0x00 */
	if (reg & SPI_CR1_SPE_Msk) Serial.println("\tSPE");
	if (reg & SPI_CR1_MASRX_Msk) Serial.println("\tMASRX");
	if (reg & SPI_CR1_CSTART_Msk) Serial.println("\tCSTART");
	if (reg & SPI_CR1_CSUSP_Msk) Serial.println("\tCSUSP");
	if (reg & SPI_CR1_HDDIR_Msk) Serial.println("\tHDDIR");
	if (reg & SPI_CR1_SSI_Msk) Serial.println("\tSSI");
	if (reg & SPI_CR1_CRC33_17_Msk) Serial.println("\tCRC33_17");
	if (reg & SPI_CR1_RCRCINI_Msk) Serial.println("\tRCRCINI");
	if (reg & SPI_CR1_TCRCINI_Msk) Serial.println("\tTCRCINI");
	if (reg & SPI_CR1_IOLOCK_Msk) Serial.println("\tIOLOCK");

	reg = pgigaSpi->CR2;
	Serial.print("  CR2:  ");
	Serial.println(reg, HEX);
	if (reg & SPI_CR2_TSER_Msk) {
		Serial.print("\tTSER: ");
		Serial.println((reg & SPI_CR2_TSER_Msk) << SPI_CR2_TSER_Pos, DEC);
	}
	if (reg & SPI_CR2_TSIZE_Msk) {
		Serial.print("\tTSIZE: ");
		Serial.println((reg & SPI_CR2_TSIZE_Msk) << SPI_CR2_TSIZE_Pos, DEC);
	}

	reg = pgigaSpi->CFG1;
	Serial.print("  CFG1:  ");
	Serial.println(reg, HEX); /*!< SPI Configuration register 1,                    Address offset: 0x08 */
	if (reg & SPI_CFG1_DSIZE_Msk) {
		Serial.print("\tDSIZE: ");
		Serial.println((reg & SPI_CFG1_DSIZE_Msk) >> SPI_CFG1_DSIZE_Pos, DEC);
	}
	if (reg & SPI_CFG1_FTHLV_Msk) {
		Serial.print("\tFTHLV: ");
		Serial.println((reg & SPI_CFG1_FTHLV_Msk) >> SPI_CFG1_FTHLV_Pos, DEC);
	}
	if (reg & SPI_CFG1_UDRCFG_Msk) {
		Serial.print("\tUDRCFG: ");
		Serial.println((reg & SPI_CFG1_UDRCFG_Msk) >> SPI_CFG1_UDRCFG_Pos, DEC);
	}
	if (reg & SPI_CFG1_UDRDET_Msk) {
		Serial.print("\tUDRDET: ");
		Serial.println((reg & SPI_CFG1_UDRDET_Msk) >> SPI_CFG1_UDRDET_Pos, DEC);
	}
	if (reg & SPI_CFG1_RXDMAEN_Msk) {
		Serial.print("\tRXDMAEN: ");
		Serial.println((reg & SPI_CFG1_RXDMAEN_Msk) >> SPI_CFG1_RXDMAEN_Pos, DEC);
	}
	if (reg & SPI_CFG1_TXDMAEN_Msk) {
		Serial.print("\tTXDMAEN: ");
		Serial.println((reg & SPI_CFG1_TXDMAEN_Msk) >> SPI_CFG1_TXDMAEN_Pos, DEC);
	}
	if (reg & SPI_CFG1_CRCSIZE_Msk) {
		Serial.print("\tCRCSIZE: ");
		Serial.println((reg & SPI_CFG1_CRCSIZE_Msk) >> SPI_CFG1_CRCSIZE_Pos, DEC);
	}
	if (reg & SPI_CFG1_CRCEN_Msk) {
		Serial.print("\tCRCEN: ");
		Serial.println((reg & SPI_CFG1_CRCEN_Msk) >> SPI_CFG1_CRCEN_Pos, DEC);
	}
	if (reg & SPI_CFG1_MBR_Msk) {
		Serial.print("\tMBR: ");
		Serial.println((reg & SPI_CFG1_MBR_Msk) >> SPI_CFG1_MBR_Pos, DEC);
	}

	reg = pgigaSpi->CFG2;
	Serial.print("  CFG2:  ");
	Serial.println(reg, HEX); /*!< SPI Configuration register 2,                    Address offset: 0x0C */
	if (reg & SPI_CFG2_MSSI_Msk) {
		Serial.print("\tMSSI: ");
		Serial.println((reg & SPI_CFG2_MSSI_Msk) >> SPI_CFG2_MSSI_Pos, DEC);
	}
	if (reg & SPI_CFG2_MIDI_Msk) {
		Serial.print("\tMIDI: ");
		Serial.println((reg & SPI_CFG2_MIDI_Msk) >> SPI_CFG2_MIDI_Pos, DEC);
	}
	if (reg & SPI_CFG2_IOSWP_Msk) {
		Serial.print("\tIOSWP: ");
		Serial.println((reg & SPI_CFG2_IOSWP_Msk) >> SPI_CFG2_IOSWP_Pos, DEC);
	}
	if (reg & SPI_CFG2_COMM_Msk) {
		Serial.print("\tCOMM: ");
		Serial.println((reg & SPI_CFG2_COMM_Msk) >> SPI_CFG2_COMM_Pos, DEC);
	}
	if (reg & SPI_CFG2_SP_Msk) {
		Serial.print("\tSP: ");
		Serial.println((reg & SPI_CFG2_SP_Msk) >> SPI_CFG2_SP_Pos, DEC);
	}
	if (reg & SPI_CFG2_MASTER_Msk) {
		Serial.print("\tMASTER: ");
		Serial.println((reg & SPI_CFG2_MASTER_Msk) >> SPI_CFG2_MASTER_Pos, DEC);
	}
	if (reg & SPI_CFG2_LSBFRST_Msk) {
		Serial.print("\tLSBFRST: ");
		Serial.println((reg & SPI_CFG2_LSBFRST_Msk) >> SPI_CFG2_LSBFRST_Pos, DEC);
	}
	if (reg & SPI_CFG2_CPHA_Msk) {
		Serial.print("\tCPHA: ");
		Serial.println((reg & SPI_CFG2_CPHA_Msk) >> SPI_CFG2_CPHA_Pos, DEC);
	}
	if (reg & SPI_CFG2_CPOL_Msk) {
		Serial.print("\tCPOL: ");
		Serial.println((reg & SPI_CFG2_CPOL_Msk) >> SPI_CFG2_CPOL_Pos, DEC);
	}
	if (reg & SPI_CFG2_SSM_Msk) {
		Serial.print("\tSSM: ");
		Serial.println((reg & SPI_CFG2_SSM_Msk) >> SPI_CFG2_SSM_Pos, DEC);
	}
	if (reg & SPI_CFG2_SSIOP_Msk) {
		Serial.print("\tSSIOP: ");
		Serial.println((reg & SPI_CFG2_SSIOP_Msk) >> SPI_CFG2_SSIOP_Pos, DEC);
	}
	if (reg & SPI_CFG2_SSOE_Msk) {
		Serial.print("\tSSOE: ");
		Serial.println((reg & SPI_CFG2_SSOE_Msk) >> SPI_CFG2_SSOE_Pos, DEC);
	}
	if (reg & SPI_CFG2_SSOM_Msk) {
		Serial.print("\tSSOM: ");
		Serial.println((reg & SPI_CFG2_SSOM_Msk) >> SPI_CFG2_SSOM_Pos, DEC);
	}
	if (reg & SPI_CFG2_AFCNTR_Msk) {
		Serial.print("\tAFCNTR: ");
		Serial.println((reg & SPI_CFG2_AFCNTR_Msk) >> SPI_CFG2_AFCNTR_Pos, DEC);
	}
	reg = pgigaSpi->IER;
	Serial.print("  IER:  ");
	Serial.println(reg, HEX); /*!< SPI/I2S Interrupt Enable register,               Address offset: 0x10 */
	if (reg & SPI_IER_RXPIE_Msk) { Serial.println("\tRXPIE"); }
	if (reg & SPI_IER_TXPIE_Msk) { Serial.println("\tTXPIE"); }
	if (reg & SPI_IER_DXPIE_Msk) { Serial.println("\tDXPIE"); }
	if (reg & SPI_IER_EOTIE_Msk) { Serial.println("\tEOTIE"); }
	if (reg & SPI_IER_TXTFIE_Msk) { Serial.println("\tTXTFIE"); }
	if (reg & SPI_IER_UDRIE_Msk) { Serial.println("\tUDRIE"); }
	if (reg & SPI_IER_OVRIE_Msk) { Serial.println("\tOVRIE"); }
	if (reg & SPI_IER_CRCEIE_Msk) { Serial.println("\tCRCEIE"); }
	if (reg & SPI_IER_TIFREIE_Msk) { Serial.println("\tTIFREIE"); }
	if (reg & SPI_IER_MODFIE_Msk) { Serial.println("\tMODFIE"); }
	if (reg & SPI_IER_TSERFIE_Msk) { Serial.println("\tTSERFIE"); }

	reg = pgigaSpi->SR;
	Serial.print("  SR:  ");
	Serial.println(reg, HEX); /*!< SPI/I2S Status register,                         Address offset: 0x14 */
	if (reg & SPI_SR_RXP_Msk) { Serial.println("\tRXP"); }
	if (reg & SPI_SR_TXP_Msk) { Serial.println("\tTXP"); }
	if (reg & SPI_SR_DXP_Msk) { Serial.println("\tDXP"); }
	if (reg & SPI_SR_EOT_Msk) { Serial.println("\tEOT"); }
	if (reg & SPI_SR_TXTF_Msk) { Serial.println("\tTXTF"); }
	if (reg & SPI_SR_UDR_Msk) { Serial.println("\tUDR"); }
	if (reg & SPI_SR_OVR_Msk) { Serial.println("\tOVR"); }
	if (reg & SPI_SR_CRCE_Msk) { Serial.println("\tCRCE"); }
	if (reg & SPI_SR_TIFRE_Msk) { Serial.println("\tTIFRE"); }
	if (reg & SPI_SR_MODF_Msk) { Serial.println("\tMODF"); }
	if (reg & SPI_SR_TSERF_Msk) { Serial.println("\tTSERF"); }
	if (reg & SPI_SR_SUSP_Msk) { Serial.println("\tSUSP"); }
	if (reg & SPI_SR_TXC_Msk) { Serial.println("\tTXC"); }
	if (reg & SPI_SR_RXPLVL_Msk) {
		Serial.print("\tRXPLVL:");
		Serial.println((reg & SPI_SR_RXPLVL_Msk) >> SPI_SR_RXPLVL_Pos, DEC);
	}
	if (reg & SPI_SR_RXWNE_Msk) { Serial.println("\tRXWNE"); }
	if (reg & SPI_SR_CTSIZE_Msk) { Serial.println("\tCTSIZE"); }

}

//=======================================================================
// Print DMA information
//=======================================================================

void dump_LISR_DISR_bits(uint8_t index, uint32_t reg_bits) {
	if ((reg_bits & 0x3f) == 0) return;
	Serial.print("\t");
	Serial.print(index, DEC);
	Serial.print(":");
	if (reg_bits & 0x01) Serial.print(" FEIF");
	if (reg_bits & 0x04) Serial.print(" DMEIF");
	if (reg_bits & 0x08) Serial.print(" TEIF");
	if (reg_bits & 0x10) Serial.print(" HTIF");
	if (reg_bits & 0x20) Serial.print(" TCIF");
	Serial.println();
}

void dump_dma_settings(const char *sz, DMA_TypeDef *pdma) {
	Serial.print("** ");
	Serial.print(sz);
	Serial.println(" **");
	Serial.print("  LISR: ");
	uint32_t reg = pdma->LISR;
	Serial.println(reg, HEX);
	dump_LISR_DISR_bits(0, reg);
	dump_LISR_DISR_bits(1, reg >> 6);
	dump_LISR_DISR_bits(2, reg >> 16);
	dump_LISR_DISR_bits(3, reg >> 22);
	Serial.print("  HISR: ");
	Serial.println(DMA1->HISR, HEX);
	reg = pdma->HISR;
	dump_LISR_DISR_bits(4, reg);
	dump_LISR_DISR_bits(5, reg >> 6);
	dump_LISR_DISR_bits(6, reg >> 16);
	dump_LISR_DISR_bits(7, reg >> 22);
}

//=======================================================================
// Print DMA information
//=======================================================================
void dump_dma_stream_settings(DMA_Stream_TypeDef *dmas) {
	switch ((uint32_t)dmas) {
	case DMA1_Stream0_BASE: Serial.println("\n** DMA1_Stream0 **"); break;
	case DMA1_Stream1_BASE: Serial.println("\n** DMA1_Stream1 **"); break;
	case DMA1_Stream2_BASE: Serial.println("\n** DMA1_Stream2 **"); break;
	case DMA1_Stream3_BASE: Serial.println("\n** DMA1_Stream3 **"); break;
	case DMA1_Stream4_BASE: Serial.println("\n** DMA1_Stream4 **"); break;
	case DMA1_Stream5_BASE: Serial.println("\n** DMA1_Stream5 **"); break;
	case DMA1_Stream6_BASE: Serial.println("\n** DMA1_Stream6 **"); break;
	case DMA1_Stream7_BASE: Serial.println("\n** DMA1_Stream7 **"); break;
	case DMA2_Stream0_BASE: Serial.println("\n** DMA2_Stream0 **"); break;
	case DMA2_Stream1_BASE: Serial.println("\n** DMA2_Stream1 **"); break;
	case DMA2_Stream2_BASE: Serial.println("\n** DMA2_Stream2 **"); break;
	case DMA2_Stream3_BASE: Serial.println("\n** DMA2_Stream3 **"); break;
	case DMA2_Stream4_BASE: Serial.println("\n** DMA2_Stream4 **"); break;
	case DMA2_Stream5_BASE: Serial.println("\n** DMA2_Stream5 **"); break;
	case DMA2_Stream6_BASE: Serial.println("\n** DMA2_Stream6 **"); break;
	case DMA2_Stream7_BASE: Serial.println("\n** DMA2_Stream7 **"); break;
	}
	Serial.print("  CR: ");
	uint32_t reg = dmas->CR;
	Serial.println(reg, HEX);
	if (reg & DMA_SxCR_MBURST_Msk) {
		Serial.print(" MBURST:");
		Serial.print((reg & DMA_SxCR_MBURST_Msk) >> DMA_SxCR_MBURST_Pos, HEX);
	}
	if (reg & DMA_SxCR_PBURST_Msk) {
		Serial.print(" tPBURST:");
		Serial.print((reg & DMA_SxCR_PBURST_Msk) >> DMA_SxCR_PBURST_Pos, HEX);
	}
	if (reg & DMA_SxCR_TRBUFF_Msk) {
		Serial.print(" TRBUFF:");
		Serial.print((reg & DMA_SxCR_TRBUFF_Msk) >> DMA_SxCR_TRBUFF_Pos, HEX);
	}
	if (reg & DMA_SxCR_CT_Msk) {
		Serial.print(" CT:");
		Serial.print((reg & DMA_SxCR_CT_Msk) >> DMA_SxCR_CT_Pos, HEX);
	}
	if (reg & DMA_SxCR_DBM_Msk) {
		Serial.print(" DBM:");
		Serial.print((reg & DMA_SxCR_DBM_Msk) >> DMA_SxCR_DBM_Pos, HEX);
	}
	if (reg & DMA_SxCR_PL_Msk) {
		Serial.print(" PL:");
		Serial.print((reg & DMA_SxCR_PL_Msk) >> DMA_SxCR_PL_Pos, HEX);
	}
	if (reg & DMA_SxCR_PINCOS_Msk) {
		Serial.print(" PINCOS:");
		Serial.print((reg & DMA_SxCR_PINCOS_Msk) >> DMA_SxCR_PINCOS_Pos, HEX);
	}
	if (reg & DMA_SxCR_MSIZE_Msk) {
		Serial.print(" MSIZE:");
		Serial.print((reg & DMA_SxCR_MSIZE_Msk) >> DMA_SxCR_MSIZE_Pos, HEX);
	}
	if (reg & DMA_SxCR_PSIZE_Msk) {
		Serial.print(" PSIZE:");
		Serial.print((reg & DMA_SxCR_PSIZE_Msk) >> DMA_SxCR_PSIZE_Pos, HEX);
	}
	if (reg & DMA_SxCR_MINC_Msk) {
		Serial.print(" MINC:");
		Serial.print((reg & DMA_SxCR_MINC_Msk) >> DMA_SxCR_MINC_Pos, HEX);
	}
	if (reg & DMA_SxCR_PINC_Msk) {
		Serial.print(" PINC:");
		Serial.print((reg & DMA_SxCR_PINC_Msk) >> DMA_SxCR_PINC_Pos, HEX);
	}
	if (reg & DMA_SxCR_CIRC_Msk) {
		Serial.print(" CIRC:");
		Serial.print((reg & DMA_SxCR_CIRC_Msk) >> DMA_SxCR_CIRC_Pos, HEX);
	}
	if (reg & DMA_SxCR_DIR_Msk) {
		Serial.print(" DIR:");
		Serial.print((reg & DMA_SxCR_DIR_Msk) >> DMA_SxCR_DIR_Pos, HEX);
	}
	if (reg & DMA_SxCR_PFCTRL_Msk) {
		Serial.print(" PFCTRL:");
		Serial.print((reg & DMA_SxCR_PFCTRL_Msk) >> DMA_SxCR_PFCTRL_Pos, HEX);
	}
	if (reg & DMA_SxCR_TCIE_Msk) {
		Serial.print(" TCIE:");
		Serial.print((reg & DMA_SxCR_TCIE_Msk) >> DMA_SxCR_TCIE_Pos, HEX);
	}
	if (reg & DMA_SxCR_HTIE_Msk) {
		Serial.print(" HTIE:");
		Serial.print((reg & DMA_SxCR_HTIE_Msk) >> DMA_SxCR_HTIE_Pos, HEX);
	}
	if (reg & DMA_SxCR_TEIE_Msk) {
		Serial.print(" TEIE:");
		Serial.print((reg & DMA_SxCR_TEIE_Msk) >> DMA_SxCR_TEIE_Pos, HEX);
	}
	if (reg & DMA_SxCR_DMEIE_Msk) {
		Serial.print(" DMEIE:");
		Serial.print((reg & DMA_SxCR_DMEIE_Msk) >> DMA_SxCR_DMEIE_Pos, HEX);
	}
	if (reg & DMA_SxCR_EN_Msk) {
		Serial.print(" EN:");
		Serial.print((reg & DMA_SxCR_EN_Msk) >> DMA_SxCR_EN_Pos, HEX);
	}


	Serial.print("\n  NDTR: ");
	Serial.print(dmas->NDTR, DEC);
	Serial.print("(0x");
	Serial.print(dmas->NDTR, HEX);
	Serial.print(")\n  PAR: ");
	Serial.println(dmas->PAR, HEX);
	Serial.print("  M0AR: ");
	Serial.println(dmas->M0AR, HEX);
	Serial.print("  M1AR: ");
	Serial.println(dmas->M1AR, HEX);
	Serial.print("  FCR: ");
	Serial.print(dmas->FCR, HEX);
	reg = dmas->FCR;
	if (reg & DMA_SxFCR_FEIE_Msk) {
		Serial.print(" FEIE:");
		Serial.print((reg & DMA_SxFCR_FEIE_Msk) >> DMA_SxFCR_FEIE_Pos, HEX);
	}
	if (reg & DMA_SxFCR_FS_Msk) {
		Serial.print(" FS:");
		Serial.print((reg & DMA_SxFCR_FS_Msk) >> DMA_SxFCR_FS_Pos, HEX);
	}
	if (reg & DMA_SxFCR_DMDIS_Msk) {
		Serial.print(" DMDIS:");
		Serial.print((reg & DMA_SxFCR_DMDIS_Msk) >> DMA_SxFCR_DMDIS_Pos, HEX);
	}
	if (reg & DMA_SxFCR_FTH_Msk) {
		Serial.print(" FTH:");
		Serial.print((reg & DMA_SxFCR_FTH_Msk) >> DMA_SxFCR_FTH_Pos, HEX);
	}
	Serial.println();
}

//=======================================================================
// Print DMAMUX information
//=======================================================================
void dump_dmamux(DMAMUX_Channel_TypeDef *dmuxc) {

	switch ((uint32_t)dmuxc) {
	case DMAMUX1_Channel0_BASE: Serial.println("\n** DMAMUX1_Channel0 **"); break;
	case DMAMUX1_Channel1_BASE: Serial.println("\n** DMAMUX1_Channel1 **"); break;
	case DMAMUX1_Channel2_BASE: Serial.println("\n** DMAMUX1_Channel2 **"); break;
	case DMAMUX1_Channel3_BASE: Serial.println("\n** DMAMUX1_Channel3 **"); break;
	case DMAMUX1_Channel4_BASE: Serial.println("\n** DMAMUX1_Channel4 **"); break;
	case DMAMUX1_Channel5_BASE: Serial.println("\n** DMAMUX1_Channel5 **"); break;
	case DMAMUX1_Channel6_BASE: Serial.println("\n** DMAMUX1_Channel6 **"); break;
	case DMAMUX1_Channel7_BASE: Serial.println("\n** DMAMUX1_Channel7 **"); break;
	case DMAMUX2_Channel0_BASE: Serial.println("\n** DMAMUX2_Channel0 **"); break;
	case DMAMUX2_Channel1_BASE: Serial.println("\n** DMAMUX2_Channel1 **"); break;
	case DMAMUX2_Channel2_BASE: Serial.println("\n** DMAMUX2_Channel2 **"); break;
	case DMAMUX2_Channel3_BASE: Serial.println("\n** DMAMUX2_Channel3 **"); break;
	case DMAMUX2_Channel4_BASE: Serial.println("\n** DMAMUX2_Channel4 **"); break;
	case DMAMUX2_Channel5_BASE: Serial.println("\n** DMAMUX2_Channel5 **"); break;
	case DMAMUX2_Channel6_BASE: Serial.println("\n** DMAMUX2_Channel6 **"); break;
	case DMAMUX2_Channel7_BASE: Serial.println("\n** DMAMUX2_Channel7 **"); break;
	}
	uint32_t reg = dmuxc->CCR;
	Serial.print("  CCR: ");
	Serial.print(reg, HEX);
	if (reg & (1 << 8)) Serial.print(" SOIE");
	if (reg & (1 << 9)) Serial.print(" EGE");
	if (reg & (1 << 16)) Serial.print(" SE");
	if (reg & (3 << 17)) {
		Serial.print(" SPOL(");
		switch ((reg >> 17) & 0x3) {
		default: break;
		case 1: Serial.print("Rise)"); break;
		case 2: Serial.print("Fall)"); break;
		case 3: Serial.print("Rise fall)"); break;
		}
	}
	Serial.print(" NBREQ(");
	Serial.print((reg >> 19) & 0x1f, DEC);
	Serial.print(") Sync(");
	Serial.print((reg >> 24) & 0x7, DEC);
	Serial.println(")");
	// Request ID:
	Serial.print("\tREQ ID: ");
	switch (reg & 0x7f) {
	case 1: Serial.println("DMAMUX1_REQ_GEN0"); break;
	case 2: Serial.println("DMAMUX1_REQ_GEN1"); break;
	case 3: Serial.println("DMAMUX1_REQ_GEN2"); break;
	case 4: Serial.println("DMAMUX1_REQ_GEN3"); break;
	case 5: Serial.println("DMAMUX1_REQ_GEN4"); break;
	case 6: Serial.println("DMAMUX1_REQ_GEN5"); break;
	case 7: Serial.println("DMAMUX1_REQ_GEN6"); break;
	case 8: Serial.println("DMAMUX1_REQ_GEN7"); break;
	case 9: Serial.println("ADC1_DMA"); break;
	case 10: Serial.println("ADC2_DMA"); break;
	case 11: Serial.println("TIM1_CH1"); break;
	case 12: Serial.println("TIM1_CH2"); break;
	case 13: Serial.println("TIM1_CH3"); break;
	case 14: Serial.println("TIM1_CH4"); break;
	case 15: Serial.println("TIM1_UP"); break;
	case 16: Serial.println("TIM1_TRIG"); break;
	case 17: Serial.println("TIM1_COM"); break;
	case 18: Serial.println("TIM2_CH1"); break;
	case 19: Serial.println("TIM2_CH2"); break;
	case 20: Serial.println("TIM2_CH3"); break;
	case 21: Serial.println("TIM2_CH4"); break;
	case 22: Serial.println("TIM2_UP"); break;
	case 23: Serial.println("TIM3_CH1"); break;
	case 24: Serial.println("TIM3_CH2"); break;
	case 25: Serial.println("TIM3_CH3"); break;
	case 26: Serial.println("TIM3_CH4"); break;
	case 27: Serial.println("TIM3_UP"); break;
	case 28: Serial.println("TIM3_TRIG"); break;
	case 29: Serial.println("TIM4_CH1"); break;
	case 30: Serial.println("TIM4_CH2"); break;
	case 31: Serial.println("TIM4_CH3"); break;
	case 32: Serial.println("TIM4_UP"); break;
	case 33: Serial.println("I2C1_RX_DMA"); break;
	case 34: Serial.println("I2C1_TX_DMA"); break;
	case 35: Serial.println("I2C2_RX_DMA"); break;
	case 36: Serial.println("I2C2_TX_DMA"); break;
	case 37: Serial.println("SPI1_RX_DMA"); break;
	case 38: Serial.println("SPI1_TX_DMA"); break;
	case 39: Serial.println("SPI2_RX_DMA"); break;
	case 40: Serial.println("SPI2_TX_DMA"); break;
	case 41: Serial.println("USART1_RX_DMA"); break;
	case 42: Serial.println("USART1_TX_DMA"); break;
	case 43: Serial.println("USART2_RX_DMA"); break;
	case 44: Serial.println("USART2_TX_DMA"); break;
	case 45: Serial.println("USART3_RX_DMA"); break;
	case 46: Serial.println("USART3_TX_DMA"); break;
	case 47: Serial.println("TIM8_CH1"); break;
	case 48: Serial.println("TIM8_CH2"); break;
	case 49: Serial.println("TIM8_CH3"); break;
	case 50: Serial.println("TIM8_CH4"); break;
	case 51: Serial.println("TIM8_UP"); break;
	case 52: Serial.println("TIM8_TRIG"); break;
	case 53: Serial.println("TIM8_COM"); break;
	case 54: Serial.println("RESERVED"); break;
	case 55: Serial.println("TIM5_CH1"); break;
	case 56: Serial.println("TIM5_CH2"); break;
	case 57: Serial.println("TIM5_CH3"); break;
	case 58: Serial.println("TIM5_CH4"); break;
	case 59: Serial.println("TIM5_UP"); break;
	case 60: Serial.println("TIM5_TRIG"); break;
	case 61: Serial.println("SPI3_RX_DMA"); break;
	case 62: Serial.println("SPI3_TX_DMA"); break;
	case 63: Serial.println("UART4_RX_DMA"); break;
	case 64: Serial.println("UART4_TX_DMA"); break;
	case 65: Serial.println("UART5_RX_DMA"); break;
	case 66: Serial.println("UART5_TX_DMA"); break;
	case 67: Serial.println("DAC_CH1_DMA"); break;
	case 68: Serial.println("DAC_CH2_DMA"); break;
	case 69: Serial.println("TIM6_UP"); break;
	case 70: Serial.println("TIM7_UP"); break;
	case 71: Serial.println("USART6_RX_DMA"); break;
	case 72: Serial.println("USART6_TX_DMA"); break;
	case 73: Serial.println("I2C3_RX_DMA"); break;
	case 74: Serial.println("I2C3_TX_DMA"); break;
	case 75: Serial.println("DCMI_DMA"); break;
	case 76: Serial.println("CRYP_IN_DMA"); break;
	case 77: Serial.println("CRYP_OUT_DMA"); break;
	case 78: Serial.println("HASH_IN_DMA"); break;
	case 79: Serial.println("UART7_RX_DMA"); break;
	case 80: Serial.println("UART7_TX_DMA"); break;
	case 81: Serial.println("UART8_RX_DMA"); break;
	case 82: Serial.println("UART8_TX_DMA"); break;
	case 83: Serial.println("SPI4_RX_DMA"); break;
	case 84: Serial.println("SPI4_TX_DMA"); break;
	case 85: Serial.println("SPI5_RX_DMA"); break;
	case 86: Serial.println("SPI5_TX_DMA"); break;
	case 87: Serial.println("SAI1A_DMA"); break;
	case 88: Serial.println("SAI1B_DMA"); break;
	case 89: Serial.println("SAI2A_DMA"); break;
	case 90: Serial.println("SAI2B_DMA"); break;
	case 91: Serial.println("SWPMI_RX_DMA"); break;
	case 92: Serial.println("SWPMI_TX_DMA"); break;
	case 93: Serial.println("SPDIFRX_DAT_DMA"); break;
	case 94: Serial.println("SPDIFRX_CTRL_DMA"); break;
	case 95: Serial.println("HR_REQ_1"); break;
	case 96: Serial.println("HR_REQ_2"); break;
	case 97: Serial.println("HR_REQ_3"); break;
	case 98: Serial.println("HR_REQ_4"); break;
	case 99: Serial.println("HR_REQ_5"); break;
	case 100: Serial.println("HR_REQ_6"); break;
	case 101: Serial.println("DFSDM1_DMA0"); break;
	case 102: Serial.println("DFSDM1_DMA1"); break;
	case 103: Serial.println("DFSDM1_DMA2"); break;
	case 104: Serial.println("DFSDM1_DMA3"); break;
	case 105: Serial.println("TIM15_CH1"); break;
	case 106: Serial.println("TIM15_UP"); break;
	case 107: Serial.println("TIM15_TRIG"); break;
	case 108: Serial.println("TIM15_COM"); break;
	case 109: Serial.println("TIM16_CH1"); break;
	case 110: Serial.println("TIM16_UP"); break;
	case 111: Serial.println("TIM17_CH1"); break;
	case 112: Serial.println("TIM17_UP"); break;
	case 113: Serial.println("SAI3_A_DMA"); break;
	case 114: Serial.println("SAI3_B_DMA"); break;
	case 115: Serial.println("ADC3_DMA"); break;
	}
}

