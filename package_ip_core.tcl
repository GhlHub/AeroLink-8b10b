set script_dir [file dirname [file normalize [info script]]]
set rtl_dir [file join $script_dir rtl]
set ip_root [file join $script_dir ip_repo aerolink_8b10b_axil]
set component_xml [file join $ip_root component.xml]

if {![file exists $component_xml]} {
    file mkdir $ip_root
    create_project -in_memory aerolink_ip_pack
    add_files -norecurse [glob -nocomplain [file join $rtl_dir *.v]]
    set_property top aerolink_axil_top [current_fileset]
    update_compile_order -fileset sources_1

    ipx::package_project \
        -root_dir $ip_root \
        -vendor user.org \
        -library user \
        -taxonomy /UserIP \
        -import_files \
        -set_current true

    set core [ipx::current_core]
    set_property name aerolink_8b10b_axil $core
    set_property display_name {AeroLink 8b10b AXI-Lite Controller} $core
    set_property description {AXI-Lite controlled AeroLink 8b10b half-duplex serial protocol IP with dual-priority FIFOs, CRC, and interrupt support.} $core
    set_property version 1.0 $core
    set_property core_revision 1 $core
    set_property supported_families {artix7 Production kintex7 Production virtex7 Production zynq Production zynquplus Production spartanuplus Production versal Production} $core

    ipx::associate_bus_interfaces -busif s_axi -clock s_axi_aclk $core

    set clk_if [ipx::get_bus_interfaces s_axi_aclk -of_objects $core]
    if {[llength $clk_if] > 0} {
        set clk_param [ipx::get_bus_parameters FREQ_HZ -of_objects $clk_if]
        if {[llength $clk_param] == 0} {
            set clk_param [ipx::add_bus_parameter FREQ_HZ $clk_if]
        }
        set_property value 125000000 $clk_param
    }

    # =========================================================================
    # Configure user parameters with human-friendly GUI presentation
    # =========================================================================

    # NUM_PORTS
    set p_num_ports [ipx::get_user_parameters NUM_PORTS -of_objects $core]
    if {[llength $p_num_ports] > 0} {
        set_property value 1 $p_num_ports
        set_property value_resolve_type user $p_num_ports
        set_property display_name {Number of AeroLink Ports} $p_num_ports

        set p_num_ports_hdl [ipx::get_hdl_parameters NUM_PORTS -of_objects $core]
        set_property value 1 $p_num_ports_hdl
        set_property value_resolve_type user $p_num_ports_hdl
    }

    # CLK_FREQ_HZ
    set p_clk [ipx::get_user_parameters CLK_FREQ_HZ -of_objects $core]
    if {[llength $p_clk] > 0} {
        set_property value 125000000 $p_clk
        set_property value_resolve_type user $p_clk
        set_property display_name {AXI Clock Frequency (Hz)} $p_clk

        set p_clk_hdl [ipx::get_hdl_parameters CLK_FREQ_HZ -of_objects $core]
        set_property value 125000000 $p_clk_hdl
        set_property value_resolve_type user $p_clk_hdl
    }

    # SYMBOL_RATE
    set p_sym [ipx::get_user_parameters SYMBOL_RATE -of_objects $core]
    if {[llength $p_sym] > 0} {
        set_property value 2500000 $p_sym
        set_property value_resolve_type user $p_sym
        set_property display_name {Symbol Rate (sym/s)} $p_sym

        set p_sym_hdl [ipx::get_hdl_parameters SYMBOL_RATE -of_objects $core]
        set_property value 2500000 $p_sym_hdl
        set_property value_resolve_type user $p_sym_hdl
    }

    ipx::create_xgui_files $core
    ipx::update_checksums $core
    ipx::save_core $core
    close_project
}

# =========================================================================
# Re-open and finalize
# =========================================================================
set core [ipx::open_core $component_xml]
set_property name aerolink_8b10b_axil $core
set_property display_name {AeroLink 8b10b AXI-Lite Controller} $core
set_property description {AXI-Lite controlled AeroLink 8b10b half-duplex serial protocol IP with dual-priority FIFOs, CRC, and interrupt support.} $core
set_property version 1.0 $core
set_property core_revision 1 $core
set_property supported_families {artix7 Production kintex7 Production virtex7 Production zynq Production zynquplus Production spartanuplus Production versal Production} $core

set clk_if [ipx::get_bus_interfaces s_axi_aclk -of_objects $core]
if {[llength $clk_if] > 0} {
    set clk_param [ipx::get_bus_parameters FREQ_HZ -of_objects $clk_if]
    if {[llength $clk_param] == 0} {
        set clk_param [ipx::add_bus_parameter FREQ_HZ $clk_if]
    }
    set_property value 125000000 $clk_param
}

ipx::update_checksums $core
ipx::check_integrity $core
ipx::save_core $core
