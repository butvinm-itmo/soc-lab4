####################################################################################
## Lab 4: Heterogeneous SoC Architecture
## Constraints File for Nexys4 DDR Board (xc7a100tcsg324-1)
####################################################################################

## Bitstream Configuration
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
