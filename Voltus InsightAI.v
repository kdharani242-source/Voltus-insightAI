// Code your design here
// MODULE 1 : FULL ADDER  (Design Under Analysis)
// The circuit whose switching activity drives the power
// integrity analysis pipeline — as required by the spec.
module full_adder (
    input  wire a,
    input  wire b,
    input  wire cin,
    output wire sum,
    output wire cout
);
    assign sum  = a ^ b ^ cin;
    assign cout = (a & b) | (b & cin) | (a & cin);
endmodule
 

// MODULE 2 : SWITCHING ACTIVITY MONITOR
// Observes Full Adder outputs over a configurable window and
// produces a 4-bit normalised activity index (0-15).
// This replaces the single binary sensor of v1.
module activity_monitor #(
    parameter WINDOW = 8
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        sum,
    input  wire        cout,
    output reg  [3:0]  activity_index
);
    reg [3:0] toggle_cnt;
    reg [3:0] cycle_cnt;
    reg       prev_sum, prev_cout;
 
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            toggle_cnt     <= 4'd0;
            cycle_cnt      <= 4'd0;
            activity_index <= 4'd0;
            prev_sum       <= 1'b0;
            prev_cout      <= 1'b0;
        end else begin
            if (sum  != prev_sum)  toggle_cnt <= toggle_cnt + 4'd1;
            if (cout != prev_cout) toggle_cnt <= toggle_cnt + 4'd1;
            prev_sum  <= sum;
            prev_cout <= cout;
            cycle_cnt <= cycle_cnt + 4'd1;
 
            if (cycle_cnt == WINDOW - 1) begin
                activity_index <= toggle_cnt;
                toggle_cnt     <= 4'd0;
                cycle_cnt      <= 4'd0;
            end
        end
    end
endmodule
 
 
// MODULE 3 : POWER GRID FSM
// Four-state FSM (IDLE/MONITOR/PROCESS/ALERT) driven by:
//   - 8-bit pg_voltage bus  : per-node voltage health flags
//   - 8-bit pg_current bus  : per-node current health flags
//   - 4-bit activity_index  : from activity_monitor
//
// Per-state power weights model relative dynamic current draw.
// Accumulated load register tracks sustained stress; ALERT
// fires when load exceeds parameterisable threshold.
module power_grid_fsm #(
    parameter BUDGET_INIT    = 8'd200,
    parameter LOAD_ALERT_THR = 8'd180,
    parameter LOAD_WARN_THR  = 8'd120
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [7:0]  pg_voltage,
    input  wire [7:0]  pg_current,
    input  wire [3:0]  activity,
    output reg  [1:0]  state,
    output reg         alarm,
    output reg         process_done,
    output reg  [7:0]  toggle_budget,
    output reg  [7:0]  accum_load,
    output reg  [3:0]  power_level,
    output reg  [2:0]  violation_nodes
);
    // State encoding
    localparam IDLE    = 2'b00;
    localparam MONITOR = 2'b01;
    localparam PROCESS = 2'b10;
    localparam ALERT   = 2'b11;
 
    // Per-state power weights
    localparam [3:0] W_IDLE    = 4'd1;
    localparam [3:0] W_MONITOR = 4'd4;
    localparam [3:0] W_PROCESS = 4'd9;
    localparam [3:0] W_ALERT   = 4'd2;
 
    // Activity thresholds
    localparam [3:0] ACT_HIGH  = 4'd10;
    localparam [3:0] ACT_MED   = 4'd5;
 
    // Count drooping nodes from pg_voltage bus
    wire [2:0] drooping;
    assign drooping =
        (~pg_voltage[0]) + (~pg_voltage[1]) +
        (~pg_voltage[2]) + (~pg_voltage[3]) +
        (~pg_voltage[4]) + (~pg_voltage[5]) +
        (~pg_voltage[6]) + (~pg_voltage[7]);
 
    // Isolated activity
    wire [3:0] act_iso   = enable ? activity : 4'd0;
    wire       high_act  = (act_iso >= ACT_HIGH) || (drooping >= 3'd4);
    wire       med_act   = (act_iso >= ACT_MED)  || (drooping >= 3'd2);
 
    // FSM sequential logic
    always @(posedge clk or posedge rst) begin
        if (rst) state <= IDLE;
        else case (state)
            IDLE:    state <= high_act                        ? PROCESS :
                              med_act                         ? MONITOR : IDLE;
            MONITOR: state <= high_act                        ? PROCESS :
                              !med_act                        ? IDLE    : MONITOR;
            PROCESS: state <= (accum_load >= LOAD_ALERT_THR)  ? ALERT   :
                              (!med_act && !high_act)          ? IDLE    : PROCESS;
            ALERT:   state <= IDLE;
            default: state <= IDLE;
        endcase
    end
 
    // Accumulated load and toggle budget
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            accum_load      <= 8'h00;
            toggle_budget   <= BUDGET_INIT;
            violation_nodes <= 3'd0;
        end else if (state == ALERT) begin
            accum_load      <= 8'h00;
            toggle_budget   <= (toggle_budget > W_ALERT) ?
                                toggle_budget - W_ALERT : 8'h00;
            violation_nodes <= drooping;
        end else begin
            case (state)
                IDLE:    accum_load <= accum_load + W_IDLE;
                MONITOR: accum_load <= accum_load + W_MONITOR;
                PROCESS: accum_load <= accum_load + W_PROCESS;
                default: accum_load <= accum_load;
            endcase
            toggle_budget   <= (toggle_budget > {4'b0, act_iso}) ?
                                toggle_budget - {4'b0, act_iso}   : 8'h00;
            violation_nodes <= drooping;
        end
    end
 
    // Power level (combinational)
    always @(*) begin
        case (state)
            IDLE:    power_level = W_IDLE    + act_iso;
            MONITOR: power_level = W_MONITOR + act_iso;
            PROCESS: power_level = W_PROCESS + act_iso;
            ALERT:   power_level = W_ALERT;
            default: power_level = 4'd0;
        endcase
    end
 
    // Output logic (combinational)
    always @(*) begin
        alarm        = 1'b0;
        process_done = 1'b0;
        case (state)
            IDLE:    begin alarm = 1'b0; process_done = 1'b0; end
            MONITOR: begin alarm = (toggle_budget < 8'd50) ? 1'b1 : 1'b0;
                          process_done = 1'b0; end
            PROCESS: begin alarm = (accum_load >= LOAD_WARN_THR) ? 1'b1 : 1'b0;
                          process_done = 1'b1; end
            ALERT:   begin alarm = 1'b1; process_done = 1'b1; end
            default: begin alarm = 1'b0; process_done = 1'b0; end
        endcase
    end
endmodule
 

// MODULE 4 : AI INFERENCE CLASSIFIER
// Threshold-based classifier that mirrors the decision
// boundaries of a trained ML model (see voltus_ai_model.py).
//
// Inputs  : power_level, accum_load, toggle_budget,
//           violation_nodes, fsm_state
// Outputs : ir_class     (0=safe 1=moderate 2=critical)
//           em_class     (0=safe 1=moderate 2=critical)
//           thermal_class(0=safe 1=warm     2=hot)
//           opt_flags    (5-bit multi-hot optimisation vector)
//
// Thresholds are derived from representative operating points
// for advanced-node standard cells and are parameterisable
// for different technology nodes.
module ai_classifier #(
    parameter [7:0] IR_MOD_THR  = 8'd80,
    parameter [7:0] IR_CRIT_THR = 8'd140,
    parameter [3:0] EM_MOD_THR  = 4'd8,
    parameter [3:0] EM_CRIT_THR = 4'd12,
    parameter [7:0] BUD_WARN    = 8'd80,
    parameter [2:0] NODE_WARN   = 3'd2,
    parameter [2:0] NODE_CRIT   = 3'd5
)(
    input  wire [3:0]  power_level,
    input  wire [7:0]  accum_load,
    input  wire [7:0]  toggle_budget,
    input  wire [2:0]  violation_nodes,
    input  wire [1:0]  fsm_state,
    output reg  [1:0]  ir_class,
    output reg  [1:0]  em_class,
    output reg  [1:0]  thermal_class,
    output reg  [4:0]  opt_flags
);
    always @(*) begin
        // IR drop classification
        if      (accum_load >= IR_CRIT_THR || violation_nodes >= NODE_CRIT)
            ir_class = 2'd2;
        else if (accum_load >= IR_MOD_THR  || violation_nodes >= NODE_WARN)
            ir_class = 2'd1;
        else
            ir_class = 2'd0;
 
        // EM risk classification
        if      (power_level >= EM_CRIT_THR) em_class = 2'd2;
        else if (power_level >= EM_MOD_THR)  em_class = 2'd1;
        else                                  em_class = 2'd0;
 
        // Thermal classification
        if      (ir_class == 2'd2 && em_class >= 2'd1) thermal_class = 2'd2;
        else if (ir_class >= 2'd1 || em_class >= 2'd1) thermal_class = 2'd1;
        else                                             thermal_class = 2'd0;
 
        // Optimisation flags (multi-hot)
        opt_flags = 5'b00000;
        if (em_class >= 2'd1)         opt_flags = opt_flags | 5'b00001; // clock gating
        if (ir_class >= 2'd1)         opt_flags = opt_flags | 5'b00010; // operand isolation
        if (ir_class == 2'd2)         opt_flags = opt_flags | 5'b00100; // decoupling caps
        if (em_class == 2'd2)         opt_flags = opt_flags | 5'b01000; // multi-Vt swap
        if (toggle_budget < BUD_WARN) opt_flags = opt_flags | 5'b10000; // widen rails
    end
endmodule
 

// MODULE 5 : TOP-LEVEL INTEGRATION
// Connects the full pipeline:
//   Full Adder -> Activity Monitor -> Power Grid FSM
//             -> AI Classifier
// All parameters exposed at top level for easy retargeting.
module voltus_insightai_top #(
    parameter ACTIVITY_WINDOW = 8,
    parameter BUDGET_INIT     = 200,
    parameter LOAD_ALERT_THR  = 180,
    parameter LOAD_WARN_THR   = 120,
    parameter IR_MOD_THR      = 80,
    parameter IR_CRIT_THR     = 140,
    parameter EM_MOD_THR      = 8,
    parameter EM_CRIT_THR     = 12,
    parameter BUD_WARN        = 80,
    parameter NODE_WARN       = 2,
    parameter NODE_CRIT       = 5
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire        fa_a,
    input  wire        fa_b,
    input  wire        fa_cin,
    input  wire [7:0]  pg_voltage,
    input  wire [7:0]  pg_current,
    output wire        fa_sum,
    output wire        fa_cout,
    output wire [1:0]  state,
    output wire        alarm,
    output wire        process_done,
    output wire [7:0]  toggle_budget,
    output wire [7:0]  accum_load,
    output wire [3:0]  power_level,
    output wire [2:0]  violation_nodes,
    output wire [1:0]  ir_class,
    output wire [1:0]  em_class,
    output wire [1:0]  thermal_class,
    output wire [4:0]  opt_flags
);
    wire [3:0] activity_index;
 
    full_adder u_fa (
        .a    (fa_a),   .b   (fa_b),
        .cin  (fa_cin), .sum (fa_sum), .cout (fa_cout)
    );
 
    activity_monitor #(.WINDOW(ACTIVITY_WINDOW)) u_am (
        .clk            (clk),  .rst (rst),
        .sum            (fa_sum), .cout (fa_cout),
        .activity_index (activity_index)
    );
 
    power_grid_fsm #(
        .BUDGET_INIT    (BUDGET_INIT),
        .LOAD_ALERT_THR (LOAD_ALERT_THR),
        .LOAD_WARN_THR  (LOAD_WARN_THR)
    ) u_fsm (
        .clk             (clk),         .rst             (rst),
        .enable          (enable),
        .pg_voltage      (pg_voltage),  .pg_current      (pg_current),
        .activity        (activity_index),
        .state           (state),       .alarm           (alarm),
        .process_done    (process_done),
        .toggle_budget   (toggle_budget),
        .accum_load      (accum_load),  .power_level     (power_level),
        .violation_nodes (violation_nodes)
    );
 
    ai_classifier #(
        .IR_MOD_THR  (IR_MOD_THR),  .IR_CRIT_THR (IR_CRIT_THR),
        .EM_MOD_THR  (EM_MOD_THR),  .EM_CRIT_THR (EM_CRIT_THR),
        .BUD_WARN    (BUD_WARN),
        .NODE_WARN   (NODE_WARN),   .NODE_CRIT   (NODE_CRIT)
    ) u_cls (
        .power_level     (power_level),
        .accum_load      (accum_load),
        .toggle_budget   (toggle_budget),
        .violation_nodes (violation_nodes),
        .fsm_state       (state),
        .ir_class        (ir_class),    .em_class        (em_class),
        .thermal_class   (thermal_class), .opt_flags     (opt_flags)
    );
endmodule

// Code your testbench here
// or browse Examples
`timescale 1ns/1ps
 
module tb_ai_fsm_power_integrity;

// TECHNOLOGY NODE PARAMETERS — change here to retarget node
parameter real SUPPLY_V      = 1.0;    // nominal supply (V)
parameter real RESISTANCE    = 0.35;   // rail resistance (Ohm)
parameter real WIRE_AREA     = 0.1;    // wire cross-section (um^2)
parameter real EM_LIMIT      = 1.5;    // EM current limit (A/um^2)
parameter real IR_WARN_MV    = 30.0;   // IR drop warning  (mV)
parameter real IR_CRIT_MV    = 80.0;   // IR drop critical (mV)
parameter      CLK_PERIOD    = 10;     // clock period (ns)

// DUT PORTS
reg        clk, rst, enable;
reg        fa_a, fa_b, fa_cin;
reg [7:0]  pg_voltage, pg_current;
 
wire       fa_sum, fa_cout;
wire [1:0] state;
wire       alarm, process_done;
wire [7:0] toggle_budget, accum_load;
wire [3:0] power_level;
wire [2:0] violation_nodes;
wire [1:0] ir_class, em_class, thermal_class;
wire [4:0] opt_flags;

// ANALYSIS VARIABLES
integer fa_toggle_count, state_trans_count;
real    current_est, ir_drop_mv, final_voltage, em_density;
real    dynamic_power_uw, leakage_power_uw;
reg [1:0] prev_state;
reg       prev_fa_sum, prev_fa_cout;

// DUT INSTANTIATION
voltus_insightai_top #(
    .ACTIVITY_WINDOW (8),
    .BUDGET_INIT     (200),
    .LOAD_ALERT_THR  (180),
    .LOAD_WARN_THR   (120),
    .IR_MOD_THR      (80),
    .IR_CRIT_THR     (140),
    .EM_MOD_THR      (8),
    .EM_CRIT_THR     (12),
    .BUD_WARN        (80),
    .NODE_WARN       (2),
    .NODE_CRIT       (5)
) uut (
    .clk             (clk),
    .rst             (rst),
    .enable          (enable),
    .fa_a            (fa_a),
    .fa_b            (fa_b),
    .fa_cin          (fa_cin),
    .pg_voltage      (pg_voltage),
    .pg_current      (pg_current),
    .fa_sum          (fa_sum),
    .fa_cout         (fa_cout),
    .state           (state),
    .alarm           (alarm),
    .process_done    (process_done),
    .toggle_budget   (toggle_budget),
    .accum_load      (accum_load),
    .power_level     (power_level),
    .violation_nodes (violation_nodes),
    .ir_class        (ir_class),
    .em_class        (em_class),
    .thermal_class   (thermal_class),
    .opt_flags       (opt_flags)
);
 
// CLOCK GENERATION
initial clk = 0;
always  #(CLK_PERIOD/2) clk = ~clk;
 
// TOGGLE AND TRANSITION MONITOR
always @(posedge clk) begin
    if (!rst) begin
        if (fa_sum  != prev_fa_sum)  fa_toggle_count   = fa_toggle_count   + 1;
        if (fa_cout != prev_fa_cout) fa_toggle_count   = fa_toggle_count   + 1;
        if (state   != prev_state)   state_trans_count = state_trans_count + 1;
    end
    prev_state   <= state;
    prev_fa_sum  <= fa_sum;
    prev_fa_cout <= fa_cout;
end

// HELPER TASKS
task apply_fa;
    input a, b, cin;
    begin
        fa_a = a; fa_b = b; fa_cin = cin;
        @(posedge clk); #1;
    end
endtask
 
task set_pg;
    input [7:0] vmask, imask;
    begin pg_voltage = vmask; pg_current = imask; end
endtask
 
task print_snapshot;
    begin
        $display("  FSM State      : %0b  Alarm=%0b  Done=%0b",
                  state, alarm, process_done);
        $display("  Power Level    : %0d  Accum Load=%0d  Budget=%0d",
                  power_level, accum_load, toggle_budget);
        $display("  Drooping Nodes : %0d / 8", violation_nodes);
    end
endtask
 
task print_ai_result;
    begin
        $display("  IR Class       : %s",
            ir_class == 2'd2 ? "CRITICAL" :
            ir_class == 2'd1 ? "MODERATE" : "SAFE");
        $display("  EM Class       : %s",
            em_class == 2'd2 ? "CRITICAL" :
            em_class == 2'd1 ? "MODERATE" : "SAFE");
        $display("  Thermal Class  : %s",
            thermal_class == 2'd2 ? "HOT"  :
            thermal_class == 2'd1 ? "WARM" : "SAFE");
        $display("  Opt Flags      : clock_gate=%0b op_iso=%0b decap=%0b multivt=%0b rail=%0b",
            opt_flags[0], opt_flags[1], opt_flags[2],
            opt_flags[3], opt_flags[4]);
    end
endtask

// MAIN SIMULATION
initial begin
 
    // Initialise
    rst = 1; enable = 0;
    fa_a = 0; fa_b = 0; fa_cin = 0;
    pg_voltage = 8'hFF; pg_current = 8'hFF;
    fa_toggle_count = 0; state_trans_count = 0;
    prev_state = 2'b00; prev_fa_sum = 0; prev_fa_cout = 0;
 
    // VCD dump — full hierarchy for ML post-processing
    $dumpfile("voltus_insightai.vcd");
    $dumpvars(0, tb_ai_fsm_power_integrity);
 
    $display("====================================================");
    $display("  VOLTUS INSIGHTAI v2.0 — POWER INTEGRITY SIM      ");
    $display("====================================================");
    $display("  Supply=%.1fV  RailR=%.2fOhm  EM_Limit=%.1fA/um2",
              SUPPLY_V, RESISTANCE, EM_LIMIT);
 
    #20; rst = 0; enable = 1;
 
    // ----------------------------------------------------------
    // SCENARIO 1 : IDLE — no switching, all nodes healthy
    // ----------------------------------------------------------
    $display("\n--- SCENARIO 1 : IDLE ---");
    set_pg(8'hFF, 8'hFF);
    repeat(10) apply_fa(0, 0, 0);
    print_snapshot(); print_ai_result();
 
    // ----------------------------------------------------------
    // SCENARIO 2 : LIGHT SWITCHING — low FA toggle rate
    // ----------------------------------------------------------
    $display("\n--- SCENARIO 2 : LIGHT SWITCHING ---");
    set_pg(8'hFF, 8'hFF);
    repeat(16) apply_fa($random % 2, 0, 0);
    print_snapshot(); print_ai_result();
 
    // ----------------------------------------------------------
    // SCENARIO 3 : MEDIUM ACTIVITY — 2 drooping nodes
    // ----------------------------------------------------------
    $display("\n--- SCENARIO 3 : MEDIUM ACTIVITY (2 drooping) ---");
    set_pg(8'b11110101, 8'hFF);
    repeat(24) apply_fa($random % 2, $random % 2, 0);
    print_snapshot(); print_ai_result();
 
    // ----------------------------------------------------------
    // SCENARIO 4 : HIGH ACTIVITY — 4 drooping nodes
    // ----------------------------------------------------------
    $display("\n--- SCENARIO 4 : HIGH ACTIVITY (4 drooping) ---");
    set_pg(8'b01010101, 8'b01010101);
    repeat(32) apply_fa($random % 2, $random % 2, $random % 2);
    print_snapshot(); print_ai_result();
 
    // ----------------------------------------------------------
    // SCENARIO 5 : STRESS BURST — worst case, 6 drooping nodes
    // ----------------------------------------------------------
    $display("\n--- SCENARIO 5 : STRESS BURST (6 drooping) ---");
    set_pg(8'b00000011, 8'b00000011);
    repeat(48) apply_fa(~fa_a, ~fa_b, ~fa_cin);
    print_snapshot(); print_ai_result();
 
    // ----------------------------------------------------------
    // SCENARIO 6 : RECOVERY — enable off, nodes recover
    // ----------------------------------------------------------
    $display("\n--- SCENARIO 6 : RECOVERY ---");
    enable = 0;
    set_pg(8'hFF, 8'hFF);
    fa_a = 0; fa_b = 0; fa_cin = 0;
    repeat(16) @(posedge clk);
    print_snapshot(); print_ai_result();
 
    // POST-SIM POWER INTEGRITY REPORT
    // All quantities derived from parameterised FSM outputs
    $display("\n====================================================");
    $display("  POST-SIMULATION POWER INTEGRITY REPORT           ");
    $display("====================================================");
 
    current_est     = power_level * 0.01;
    ir_drop_mv      = current_est * RESISTANCE * 1000.0;
    final_voltage   = SUPPLY_V - (current_est * RESISTANCE);
    em_density      = current_est / WIRE_AREA;
    dynamic_power_uw = accum_load * 0.5;
    leakage_power_uw = dynamic_power_uw * 0.05;
 
    $display("  FA Toggle Count      = %0d",    fa_toggle_count);
    $display("  State Transitions    = %0d",    state_trans_count);
    $display("  Violation Nodes      = %0d/8",  violation_nodes);
    $display("  Toggle Budget Left   = %0d",    toggle_budget);
    $display("  Accumulated Load     = %0d",    accum_load);
    $display("  Power Level Index    = %0d",    power_level);
    $display("  Estimated Current    = %.4f A", current_est);
    $display("  IR Drop              = %.2f mV",ir_drop_mv);
    $display("  Final Cell Voltage   = %.4f V", final_voltage);
    $display("  EM Current Density   = %.4f A/um2", em_density);
    $display("  Dynamic Power        = %.2f uW",dynamic_power_uw);
    $display("  Leakage Power        = %.2f uW",leakage_power_uw);
 
    $display("----------------------------------------------------");
 
    // IR verdict
    if (ir_drop_mv > IR_CRIT_MV)
        $display("  IR Analysis  : CRITICAL  (%.1f mV > %.0f mV)",
                  ir_drop_mv, IR_CRIT_MV);
    else if (ir_drop_mv > IR_WARN_MV)
        $display("  IR Analysis  : MODERATE  (%.1f mV > %.0f mV)",
                  ir_drop_mv, IR_WARN_MV);
    else
        $display("  IR Analysis  : SAFE      (%.1f mV)", ir_drop_mv);
 
    // EM verdict
    if (em_density > EM_LIMIT)
        $display("  EM Analysis  : RISK      (%.4f A/um2 > %.1f)",
                  em_density, EM_LIMIT);
    else
        $display("  EM Analysis  : SAFE      (%.4f A/um2)", em_density);
 
    $display("----------------------------------------------------");
    $display("  AI Classifier Final:");
    print_ai_result();
 
    $display("----------------------------------------------------");
    $display("  Technology Recommendation:");
    if      (power_level <= 4) $display("  -> 28nm CMOS   (low power)");
    else if (power_level <= 8) $display("  -> 12nm FinFET (moderate activity)");
    else if (power_level <=12) $display("  -> 7nm FinFET  (balanced PPA)");
    else                       $display("  -> 5nm FinFET  (high performance)");
 
    $display("====================================================");
    $display("  VCD written to voltus_insightai.vcd");
    $display("  Run: python voltus_ai_model.py voltus_insightai.vcd");
    $display("====================================================");
 
    $finish;
end
 
endmodule

//output
# KERNEL: ====================================================
# KERNEL:   VOLTUS INSIGHTAI v2.0 â€” POWER INTEGRITY SIM      
# KERNEL: ====================================================
# KERNEL:   Supply=1.0V  RailR=0.35Ohm  EM_Limit=1.5A/um2
# KERNEL: 
# KERNEL: --- SCENARIO 1 : IDLE ---
# KERNEL:   FSM State      : 0  Alarm=0  Done=0
# KERNEL:   Power Level    : 1  Accum Load=10  Budget=200
# KERNEL:   Drooping Nodes : 0 / 8
# KERNEL:   IR Class       :     SAFE
# KERNEL:   EM Class       :     SAFE
# KERNEL:   Thermal Class  : SAFE
# KERNEL:   Opt Flags      : clock_gate=0 op_iso=0 decap=0 multivt=0 rail=0
# KERNEL: 
# KERNEL: --- SCENARIO 2 : LIGHT SWITCHING ---
# KERNEL:   FSM State      : 0  Alarm=0  Done=0
# KERNEL:   Power Level    : 5  Accum Load=26  Budget=184
# KERNEL:   Drooping Nodes : 0 / 8
# KERNEL:   IR Class       :     SAFE
# KERNEL:   EM Class       :     SAFE
# KERNEL:   Thermal Class  : SAFE
# KERNEL:   Opt Flags      : clock_gate=0 op_iso=0 decap=0 multivt=0 rail=0
# KERNEL: 
# KERNEL: --- SCENARIO 3 : MEDIUM ACTIVITY (2 drooping) ---
# KERNEL:   FSM State      : 1  Alarm=0  Done=0
# KERNEL:   Power Level    : 8  Accum Load=119  Budget=56
# KERNEL:   Drooping Nodes : 2 / 8
# KERNEL:   IR Class       : MODERATE
# KERNEL:   EM Class       : MODERATE
# KERNEL:   Thermal Class  : WARM
# KERNEL:   Opt Flags      : clock_gate=1 op_iso=1 decap=0 multivt=0 rail=1
# KERNEL: 
# KERNEL: --- SCENARIO 4 : HIGH ACTIVITY (4 drooping) ---
# KERNEL:   FSM State      : 11  Alarm=1  Done=1
# KERNEL:   Power Level    : 2  Accum Load=190  Budget=0
# KERNEL:   Drooping Nodes : 4 / 8
# KERNEL:   IR Class       : CRITICAL
# KERNEL:   EM Class       :     SAFE
# KERNEL:   Thermal Class  : WARM
# KERNEL:   Opt Flags      : clock_gate=0 op_iso=1 decap=1 multivt=0 rail=1
# KERNEL: 
# KERNEL: --- SCENARIO 5 : STRESS BURST (6 drooping) ---
# KERNEL:   FSM State      : 10  Alarm=0  Done=1
# KERNEL:   Power Level    : 0  Accum Load=1  Budget=0
# KERNEL:   Drooping Nodes : 6 / 8
# KERNEL:   IR Class       : CRITICAL
# KERNEL:   EM Class       :     SAFE
# KERNEL:   Thermal Class  : WARM
# KERNEL:   Opt Flags      : clock_gate=0 op_iso=1 decap=1 multivt=0 rail=1
# KERNEL: 
# KERNEL: --- SCENARIO 6 : RECOVERY ---
# KERNEL:   FSM State      : 0  Alarm=0  Done=0
# KERNEL:   Power Level    : 1  Accum Load=24  Budget=0
# KERNEL:   Drooping Nodes : 0 / 8
# KERNEL:   IR Class       :     SAFE
# KERNEL:   EM Class       :     SAFE
# KERNEL:   Thermal Class  : SAFE
# KERNEL:   Opt Flags      : clock_gate=0 op_iso=0 decap=0 multivt=0 rail=1
# KERNEL: 
# KERNEL: ====================================================
# KERNEL:   POST-SIMULATION POWER INTEGRITY REPORT           
# KERNEL: ====================================================
# KERNEL:   FA Toggle Count      = 162
# KERNEL:   State Transitions    = 15
# KERNEL:   Violation Nodes      = 0/8
# KERNEL:   Toggle Budget Left   = 0
# KERNEL:   Accumulated Load     = 24
# KERNEL:   Power Level Index    = 1
# KERNEL:   Estimated Current    = 0.0100 A
# KERNEL:   IR Drop              = 3.50 mV
# KERNEL:   Final Cell Voltage   = 0.9965 V
# KERNEL:   EM Current Density   = 0.1000 A/um2
# KERNEL:   Dynamic Power        = 12.00 uW
# KERNEL:   Leakage Power        = 0.60 uW
# KERNEL: ----------------------------------------------------
# KERNEL:   IR Analysis  : SAFE      (3.5 mV)
# KERNEL:   EM Analysis  : SAFE      (0.1000 A/um2)
# KERNEL: ----------------------------------------------------
# KERNEL:   AI Classifier Final:
# KERNEL:   IR Class       :     SAFE
# KERNEL:   EM Class       :     SAFE
# KERNEL:   Thermal Class  : SAFE
# KERNEL:   Opt Flags      : clock_gate=0 op_iso=0 decap=0 multivt=0 rail=1
# KERNEL: ----------------------------------------------------
# KERNEL:   Technology Recommendation:
# KERNEL:   -> 28nm CMOS   (low power)
# KERNEL: ====================================================
# KERNEL:   VCD written to voltus_insightai.vcd
# KERNEL:   Run: python voltus_ai_model.py voltus_insightai.vcd
# KERNEL: ====================================================
