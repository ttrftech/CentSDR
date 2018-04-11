    .ccmfunc : ALIGN(4)
    {
        . = ALIGN(4);
        __ccmfunc_init_text__ = LOADADDR(.ccmfunc);
        __ccmfunc_init__ = .;
		*dsp.o(.text.*)
		*dsp.o(.rodata.arctantbl)
		*arm_biquad_cascade_df1_q15.o(.text.*)
		*arm_cfft_radix4_q31.o(.text.*)
		*arm_bitreversal.o(.text.*)
		*display.o(.text.draw_waveform)
		*display.o(.text.draw_waterfall)
		*display.o(.text.draw_spectrogram)
		*display.o(.text.disp_fetch_samples)
		__ccmfunc_end__ = .;
		. = ALIGN(4);
    } > ram4 AT > RAM_INIT_FLASH_LMA
