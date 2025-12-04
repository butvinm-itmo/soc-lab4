################################################################
# Lab 4: Heterogeneous SoC Architecture
# TCL Script to modify lab1 project for lab4
################################################################

set project_dir [file dirname [info script]]
set hls_ip_repo "$project_dir/../lab3/mat_comparison_pipeline/solution/impl/ip"

# Open existing project
open_project "$project_dir/project_1/project_1.xpr"

# Add HLS IP repository
set_property ip_repo_paths [list $hls_ip_repo] [current_project]
update_ip_catalog

# Upgrade all locked IPs
upgrade_ip [get_ips]

# Open block design
open_bd_design [get_files design_1.bd]

################################################################
# Modify AXI GPIO for dual-channel (add switches)
################################################################
set_property -dict [list \
    CONFIG.C_IS_DUAL {1} \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_GPIO2_WIDTH {16} \
] [get_bd_cells axi_gpio_0]

# Create external port for switches
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 dip_switches_16bits
connect_bd_intf_net [get_bd_intf_pins axi_gpio_0/GPIO2] [get_bd_intf_ports dip_switches_16bits]

################################################################
# Modify AXI Interconnect to add third master port
################################################################
set_property CONFIG.NUM_MI {3} [get_bd_cells microblaze_0_axi_periph]

################################################################
# Add HLS mat_compute IP
################################################################
create_bd_cell -type ip -vlnv xilinx.com:hls:mat_compute:1.0 mat_compute_0

# Connect clock and reset
connect_bd_net [get_bd_pins clk_wiz_1/clk_out1] [get_bd_pins mat_compute_0/ap_clk]
connect_bd_net [get_bd_pins rst_clk_wiz_1_100M/peripheral_aresetn] [get_bd_pins mat_compute_0/ap_rst_n]

# Connect AXI interface
connect_bd_intf_net [get_bd_intf_pins microblaze_0_axi_periph/M02_AXI] [get_bd_intf_pins mat_compute_0/s_axi_AXILiteS]

# Connect M02 clock and reset
connect_bd_net [get_bd_pins clk_wiz_1/clk_out1] [get_bd_pins microblaze_0_axi_periph/M02_ACLK]
connect_bd_net [get_bd_pins rst_clk_wiz_1_100M/peripheral_aresetn] [get_bd_pins microblaze_0_axi_periph/M02_ARESETN]

################################################################
# Assign address for HLS IP
################################################################
assign_bd_address [get_bd_addr_segs mat_compute_0/s_axi_AXILiteS/Reg]
set_property offset 0x44A00000 [get_bd_addr_segs {microblaze_0/Data/SEG_mat_compute_0_Reg}]
set_property range 64K [get_bd_addr_segs {microblaze_0/Data/SEG_mat_compute_0_Reg}]

################################################################
# Validate and save
################################################################
validate_bd_design
save_bd_design

# Regenerate wrapper
make_wrapper -files [get_files design_1.bd] -top -force
add_files -norecurse "$project_dir/project_1/project_1.srcs/sources_1/bd/design_1/hdl/design_1_wrapper.v"

################################################################
# Add simulation sources
################################################################
remove_files -fileset sim_1 [get_files -of_objects [get_filesets sim_1]]

add_files -fileset sim_1 -norecurse "$project_dir/sim/tb_defines.svh"
add_files -fileset sim_1 -norecurse "$project_dir/sim/dev_tb.v"
add_files -fileset sim_1 -norecurse "$project_dir/sim/agent.sv"
add_files -fileset sim_1 -norecurse "$project_dir/sim/driver.sv"
add_files -fileset sim_1 -norecurse "$project_dir/sim/sequencer.sv"
add_files -fileset sim_1 -norecurse "$project_dir/sim/monitor.sv"
add_files -fileset sim_1 -norecurse "$project_dir/sim/scoreboard.sv"

set_property top dev_tb [get_filesets sim_1]
update_compile_order -fileset sim_1

################################################################
# Add constraints
################################################################
add_files -fileset constrs_1 -norecurse "$project_dir/constraints.xdc"

################################################################
# Generate output products
################################################################
generate_target all [get_files design_1.bd]

puts "================================================================"
puts "Lab 4 Project Modified Successfully!"
puts "================================================================"

close_project
