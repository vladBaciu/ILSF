# THIS FILE IS AUTOMATICALLY GENERATED
# Project: D:\Vlad\[4]_Hospital_APP\PSOC_Project\PSOC_PPG.cydsn\PSOC_PPG.cyprj
# Date: Thu, 28 Oct 2021 08:04:12 GMT
#set_units -time ns
create_clock -name {CyILO} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyClk_LF} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/lfclk}]]
create_clock -name {CyFLL} -period 10 -waveform {0 5} [list [get_pins {ClockBlock/fll}]]
create_clock -name {CyClk_HF0} -period 10 -waveform {0 5} [list [get_pins {ClockBlock/hfclk_0}]]
create_clock -name {CyClk_Fast} -period 10 -waveform {0 5} [list [get_pins {ClockBlock/fastclk}]]
create_clock -name {CyClk_Peri} -period 20 -waveform {0 10} [list [get_pins {ClockBlock/periclk}]]
create_generated_clock -name {CyClk_Slow} -source [get_pins {ClockBlock/periclk}] -edges {1 2 3} [list [get_pins {ClockBlock/slowclk}]]
create_generated_clock -name {ADC_INVERTING_AMP_intSarClock} -source [get_pins {ClockBlock/periclk}] -edges {1 51 101} [list [get_pins {ClockBlock/ff_div_49}]]
create_generated_clock -name {Clock_1ms} -source [get_pins {ClockBlock/periclk}] -edges {1 51 101} [list [get_pins {ClockBlock/ff_div_11}] [get_pins {ClockBlock/ff_div_12}] [get_pins {ClockBlock/ff_div_13}]]
create_generated_clock -name {Uart_Printf_SCBCLK} -source [get_pins {ClockBlock/periclk}] -edges {1 37 73} [list [get_pins {ClockBlock/ff_div_5}]]
create_generated_clock -name {I2C_MAX30105_SCBCLK} -source [get_pins {ClockBlock/periclk}] -edges {1 33 65} [list [get_pins {ClockBlock/ff_div_6}]]
create_clock -name {CyPeriClk_App} -period 20 -waveform {0 10} [list [get_pins {ClockBlock/periclk_App}]]
create_clock -name {CyIMO} -period 125 -waveform {0 62.5} [list [get_pins {ClockBlock/imo}]]


# Component constraints for D:\Vlad\[4]_Hospital_APP\PSOC_Project\PSOC_PPG.cydsn\TopDesign\TopDesign.cysch
# Project: D:\Vlad\[4]_Hospital_APP\PSOC_Project\PSOC_PPG.cydsn\PSOC_PPG.cyprj
# Date: Thu, 28 Oct 2021 08:03:47 GMT