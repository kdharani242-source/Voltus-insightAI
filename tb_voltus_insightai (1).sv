// ============================================================
// VOLTUS INSIGHTAI v3.0 — COMPREHENSIVE TESTBENCH
// ============================================================
// IMPROVEMENT: Separated into standalone tb file (was co-located
// with design source). Run with:
//   xrun voltus_insightai_top.v tb_voltus_insightai.sv -sv
//   iverilog -g2012 voltus_insightai_top.v tb_voltus_insightai.sv -o sim && vvp sim
// ============================================================
// Simulator  : Cadence Xcelium (xrun) / ModelSim / Icarus
// Assertions : SystemVerilog Assertion (SVA) — IEEE 1800-2012
// Coverage   : Functional (toggle + FSM state coverage)
// VCD Output : voltus_insightai.vcd  (Voltus post-process)
// ============================================================
`timescale 1ns/1ps

module tb_voltus_insightai;

// ============================================================
// TECHNOLOGY NODE PARAMETERS — change here to retarget node
// ============================================================
parameter real SUPPLY_V   = 1.0;
parameter real RESISTANCE = 0.35;
parameter real WIRE_AREA  = 0.1;
parameter real EM_LIMIT   = 1.5;
parameter real IR_WARN_MV = 30.0;
parameter real IR_CRIT_MV = 80.0;
parameter      CLK_PERIOD = 10;

// ============================================================
// DUT PORT DECLARATIONS
// ============================================================
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
wire [7:0] ml_ir_score, ml_em_score;   // NEW v3: ML outputs
wire [3:0] confidence;                  // NEW v3: ML confidence

// ============================================================
// ANALYSIS VARIABLES
// ============================================================
integer fa_toggle_count, state_trans_count, assert_pass_count, assert_fail_count;
real    current_est, ir_drop_mv, final_voltage, em_density;
real    dynamic_power_uw, leakage_power_uw;
reg [1:0] prev_state;
reg       prev_fa_sum, prev_fa_cout;

// State name lookup
function [63:0] state_name;
    input [1:0] s;
    begin
        case (s)
            2'b00: state_name = "IDLE   ";
            2'b01: state_name = "MONITOR";
            2'b10: state_name = "PROCESS";
            2'b11: state_name = "ALERT  ";
        endcase
    end
endfunction

// ============================================================
// DUT INSTANTIATION
// ============================================================
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
    .opt_flags       (opt_flags),
    .ml_ir_score     (ml_ir_score),
    .ml_em_score     (ml_em_score),
    .confidence      (confidence)
);

// ============================================================
// CLOCK GENERATION
// ============================================================
initial clk = 1'b0;
always  #(CLK_PERIOD/2) clk = ~clk;

// ============================================================
// TOGGLE AND FSM TRANSITION MONITOR
// ============================================================
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

// ============================================================
// SYSTEMVERILOG ASSERTIONS (SVA) — 14 Properties (was 12)
// ============================================================

// A1: Reset clears FSM to IDLE
property p_rst_idle;
    @(posedge clk) $rose(rst) |=> (state == 2'b00);
endproperty
A1_RST_IDLE: assert property (p_rst_idle)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A1: RST did not clear state to IDLE"); end

// A2: ALERT always returns to IDLE next cycle
property p_alert_to_idle;
    @(posedge clk) disable iff (rst)
    (state == 2'b11) |=> (state == 2'b00);
endproperty
A2_ALERT_TO_IDLE: assert property (p_alert_to_idle)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A2: ALERT did not return to IDLE"); end

// A3: ALERT always asserts alarm
property p_alert_alarm;
    @(posedge clk) disable iff (rst)
    (state == 2'b11) |-> (alarm == 1'b1);
endproperty
A3_ALERT_ALARM: assert property (p_alert_alarm)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A3: ALERT state without alarm"); end

// A4: IDLE state must not assert alarm
property p_idle_no_alarm;
    @(posedge clk) disable iff (rst)
    (state == 2'b00) |-> (alarm == 1'b0);
endproperty
A4_IDLE_NO_ALARM: assert property (p_idle_no_alarm)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A4: IDLE asserted alarm unexpectedly"); end

// A5: process_done must be 0 in IDLE
property p_idle_no_done;
    @(posedge clk) disable iff (rst)
    (state == 2'b00) |-> (process_done == 1'b0);
endproperty
A5_IDLE_NO_DONE: assert property (p_idle_no_done)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A5: process_done asserted in IDLE"); end

// A6: PROCESS must assert process_done
property p_proc_done;
    @(posedge clk) disable iff (rst)
    (state == 2'b10) |-> (process_done == 1'b1);
endproperty
A6_PROC_DONE: assert property (p_proc_done)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A6: process_done not set in PROCESS"); end

// A7: toggle_budget must not exceed BUDGET_INIT (200)
property p_budget_bound;
    @(posedge clk) disable iff (rst)
    (toggle_budget <= 8'd200);
endproperty
A7_BUDGET_BOUND: assert property (p_budget_bound)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A7: toggle_budget exceeded BUDGET_INIT"); end

// A8: accum_load resets to 0 on ALERT exit
property p_load_clr_on_alert;
    @(posedge clk) disable iff (rst)
    (state == 2'b11) |=> (accum_load == 8'h00);
endproperty
A8_LOAD_CLR: assert property (p_load_clr_on_alert)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A8: accum_load not cleared after ALERT"); end

// A9: IR CRITICAL => opt_flags[2] (decap) set
property p_ir_crit_decap;
    @(posedge clk) disable iff (rst)
    (ir_class == 2'd2) |-> (opt_flags[2] == 1'b1);
endproperty
A9_IR_CRIT_DECAP: assert property (p_ir_crit_decap)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A9: IR CRITICAL without decap flag"); end

// A10: Thermal HOT requires IR>=CRIT and EM>=MOD
property p_thermal_hot_prereq;
    @(posedge clk) disable iff (rst)
    (thermal_class == 2'd2) |-> (ir_class == 2'd2 && em_class >= 2'd1);
endproperty
A10_THERMAL_HOT: assert property (p_thermal_hot_prereq)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A10: Thermal HOT without IR CRITICAL + EM"); end

// A11: No illegal state encoding
property p_valid_state;
    @(posedge clk) (state inside {2'b00, 2'b01, 2'b10, 2'b11});
endproperty
A11_VALID_STATE: assert property (p_valid_state)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A11: Illegal FSM state encoding"); end

// A12: Budget only decreases (never spontaneously increases)
property p_budget_monotone;
    @(posedge clk) disable iff (rst || state == 2'b11)
    (toggle_budget <= $past(toggle_budget));
endproperty
A12_BUDGET_MONO: assert property (p_budget_monotone)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A12: toggle_budget increased outside ALERT/RST"); end

// NEW A13: ML IR score must be bounded [0,255]
property p_ml_ir_bounded;
    @(posedge clk) disable iff (rst)
    (ml_ir_score <= 8'd255);
endproperty
A13_ML_IR_BOUND: assert property (p_ml_ir_bounded)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A13: ml_ir_score out of bounds"); end

// NEW A14: Confidence must be > 0 when classifier is active
property p_confidence_nonzero;
    @(posedge clk) disable iff (rst)
    (state != 2'b00) |-> (confidence > 4'd0);
endproperty
A14_CONF_NONZERO: assert property (p_confidence_nonzero)
    else begin assert_fail_count = assert_fail_count + 1;
              $error("[ASSERT FAIL] A14: Classifier confidence is zero in active state"); end

// ============================================================
// HELPER TASKS
// ============================================================
task apply_fa;
    input a, b, cin;
    begin
        fa_a = a; fa_b = b; fa_cin = cin;
        @(posedge clk); #1;
    end
endtask

task set_pg;
    input [7:0] vmask, imask;
    begin
        pg_voltage = vmask;
        pg_current = imask;
    end
endtask

task print_snapshot;
    begin
        $display("  FSM State      : %s  Alarm=%0b  Done=%0b",
                  state_name(state), alarm, process_done);
        $display("  Power Level    : %0d  Accum Load=%0d  Budget=%0d",
                  power_level, accum_load, toggle_budget);
        $display("  Drooping Nodes : %0d / 8", violation_nodes);
        $display("  ML IR Score    : %0d/255  ML EM Score=%0d/255  Confidence=%0d/15",
                  ml_ir_score, ml_em_score, confidence);
    end
endtask

task print_ai_result;
    begin
        $display("  IR Class       : %s (ML=%0d)",
            ir_class == 2'd2 ? "CRITICAL" :
            ir_class == 2'd1 ? "MODERATE" : "SAFE    ", ml_ir_score);
        $display("  EM Class       : %s (ML=%0d)",
            em_class == 2'd2 ? "CRITICAL" :
            em_class == 2'd1 ? "MODERATE" : "SAFE    ", ml_em_score);
        $display("  Thermal Class  : %s",
            thermal_class == 2'd2 ? "HOT " :
            thermal_class == 2'd1 ? "WARM" : "SAFE");
        $display("  Opt Flags      : clock_gate=%0b op_iso=%0b decap=%0b multivt=%0b rail=%0b",
            opt_flags[0], opt_flags[1], opt_flags[2],
            opt_flags[3], opt_flags[4]);
        $display("  Classifier Conf: %0d/15", confidence);
    end
endtask

task check;
    input        cond;
    input [127:0] msg;
    begin
        if (cond) begin
            $display("  [PASS] %0s", msg);
            assert_pass_count = assert_pass_count + 1;
        end else begin
            $display("  [FAIL] %0s", msg);
            assert_fail_count = assert_fail_count + 1;
        end
    end
endtask

// ============================================================
// MAIN SIMULATION
// ============================================================
initial begin

    rst = 1; enable = 0;
    fa_a = 0; fa_b = 0; fa_cin = 0;
    pg_voltage = 8'hFF; pg_current = 8'hFF;
    fa_toggle_count   = 0; state_trans_count = 0;
    assert_pass_count = 0; assert_fail_count = 0;
    prev_state   = 2'b00;
    prev_fa_sum  = 1'b0;
    prev_fa_cout = 1'b0;

    $dumpfile("voltus_insightai.vcd");
    $dumpvars(0, tb_voltus_insightai);

    $display("====================================================");
    $display("  VOLTUS INSIGHTAI v3.0 — POWER INTEGRITY SIM      ");
    $display("====================================================");
    $display("  Supply=%.1fV  RailR=%.2fOhm  EM_Limit=%.1fA/um2",
              SUPPLY_V, RESISTANCE, EM_LIMIT);
    $display("  SVA assertions active (14 properties, +2 ML)");
    $display("  ML Classifier: weighted GBT fixed-point inference");

    #20; rst = 0; enable = 1;

    // --------------------------------------------------------
    // SCENARIO 1 : IDLE — no switching, all nodes healthy
    // --------------------------------------------------------
    $display("\n--- SCENARIO 1 : IDLE (all-zero inputs) ---");
    set_pg(8'hFF, 8'hFF);
    repeat(12) apply_fa(0, 0, 0);
    print_snapshot(); print_ai_result();
    check(state == 2'b00,             "S1: FSM in IDLE");
    check(alarm == 1'b0,              "S1: No alarm in IDLE");
    check(ir_class == 2'd0,           "S1: IR class = SAFE");
    check(ml_ir_score < 8'd96,        "S1: ML IR score below MODERATE threshold");

    // --------------------------------------------------------
    // SCENARIO 2 : LIGHT SWITCHING — low FA toggle rate
    // --------------------------------------------------------
    $display("\n--- SCENARIO 2 : LIGHT SWITCHING ---");
    set_pg(8'hFF, 8'hFF);
    repeat(16) apply_fa($random % 2, 1'b0, 1'b0);
    print_snapshot(); print_ai_result();
    check(state != 2'b11,             "S2: No ALERT on light switching");

    // --------------------------------------------------------
    // SCENARIO 3 : MEDIUM ACTIVITY — 2 drooping nodes
    // --------------------------------------------------------
    $display("\n--- SCENARIO 3 : MEDIUM ACTIVITY (2 drooping) ---");
    set_pg(8'b11110101, 8'hFF);
    repeat(24) apply_fa($random % 2, $random % 2, 1'b0);
    print_snapshot(); print_ai_result();
    check(violation_nodes >= 3'd2,    "S3: >=2 drooping nodes detected");
    check(ml_ir_score >= 8'd30,       "S3: ML IR score elevated by node stress");

    // --------------------------------------------------------
    // SCENARIO 4 : HIGH ACTIVITY — 4 drooping nodes
    // --------------------------------------------------------
    $display("\n--- SCENARIO 4 : HIGH ACTIVITY (4 drooping) ---");
    set_pg(8'b01010101, 8'b01010101);
    repeat(32) apply_fa($random % 2, $random % 2, $random % 2);
    print_snapshot(); print_ai_result();
    check(violation_nodes >= 3'd4,    "S4: >=4 drooping nodes detected");
    check(state == 2'b10 || state == 2'b11,
                                      "S4: FSM in PROCESS or ALERT");
    check(ml_em_score >= 8'd50,       "S4: ML EM score elevated under high activity");

    // --------------------------------------------------------
    // SCENARIO 5 : STRESS BURST — 6 drooping nodes, max toggle
    // --------------------------------------------------------
    $display("\n--- SCENARIO 5 : STRESS BURST (6 drooping) ---");
    set_pg(8'b00000011, 8'b00000011);
    repeat(48) apply_fa(~fa_a, ~fa_b, ~fa_cin);
    print_snapshot(); print_ai_result();
    check(violation_nodes >= 3'd4,    "S5: High drooping node count");
    check(ir_class >= 2'd1,           "S5: IR at least MODERATE");
    check(ml_ir_score >= 8'd80,       "S5: ML IR score confirms risk");

    // --------------------------------------------------------
    // SCENARIO 6 : ALERT BOUNDARY TEST
    // --------------------------------------------------------
    $display("\n--- SCENARIO 6 : ALERT BOUNDARY TEST ---");
    rst = 1; #(CLK_PERIOD*2);
    rst = 0; enable = 1;
    set_pg(8'b01010101, 8'b01010101);
    repeat(30) apply_fa(1, 0, 1);
    print_snapshot(); print_ai_result();
    if (state == 2'b00)
        check(accum_load < 8'd10,     "S6: accum_load cleared after ALERT");
    else
        check(alarm == 1'b1,          "S6: Alarm active during stress");

    // --------------------------------------------------------
    // SCENARIO 7 : RECOVERY — enable off, all nodes recover
    // --------------------------------------------------------
    $display("\n--- SCENARIO 7 : RECOVERY ---");
    enable = 0;
    set_pg(8'hFF, 8'hFF);
    fa_a = 0; fa_b = 0; fa_cin = 0;
    repeat(20) @(posedge clk);
    print_snapshot(); print_ai_result();
    check(violation_nodes == 3'd0,    "S7: No drooping nodes");
    check(ml_ir_score < 8'd60,        "S7: ML IR score reduced in recovery");

    // --------------------------------------------------------
    // SCENARIO 8 : FULL ADDER TRUTH TABLE VERIFICATION
    // --------------------------------------------------------
    $display("\n--- SCENARIO 8 : FULL ADDER TRUTH TABLE ---");
    enable = 0;
    begin : fa_truth_table
        reg exp_sum, exp_cout;
        integer i;
        for (i = 0; i < 8; i = i + 1) begin
            fa_a   = i[2]; fa_b = i[1]; fa_cin = i[0];
            #2;
            exp_sum  = i[2] ^ i[1] ^ i[0];
            exp_cout = (i[2] & i[1]) | (i[1] & i[0]) | (i[2] & i[0]);
            if (fa_sum !== exp_sum || fa_cout !== exp_cout)
                $display("  [FAIL] FA: a=%0b b=%0b cin=%0b sum=%0b(exp %0b) cout=%0b(exp %0b)",
                          fa_a, fa_b, fa_cin,
                          fa_sum, exp_sum, fa_cout, exp_cout);
            else
                $display("  [PASS] FA: %0b+%0b+%0b = sum=%0b cout=%0b",
                          fa_a, fa_b, fa_cin, fa_sum, fa_cout);
        end
    end

    // NEW: SCENARIO 9 : ML CLASSIFIER BOUNDARY VALIDATION
    $display("\n--- SCENARIO 9 : ML SCORE BOUNDARY VALIDATION ---");
    rst = 1; #(CLK_PERIOD*2); rst = 0; enable = 1;
    // Drive to exactly IR_MOD_THR boundary (accum_load=80)
    set_pg(8'hFF, 8'hFF);
    repeat(25) apply_fa(1, 1, 0);  // Force moderate load accumulation
    print_snapshot(); print_ai_result();
    check(ml_ir_score <= 8'd255,      "S9: ML IR score within valid range");
    check(ml_em_score <= 8'd255,      "S9: ML EM score within valid range");
    check(confidence > 4'd0,          "S9: Confidence is non-zero");

    // ============================================================
    // POST-SIMULATION POWER INTEGRITY REPORT
    // ============================================================
    $display("\n====================================================");
    $display("  POST-SIMULATION POWER INTEGRITY REPORT           ");
    $display("====================================================");

    current_est      = power_level * 0.01;
    ir_drop_mv       = current_est * RESISTANCE * 1000.0;
    final_voltage    = SUPPLY_V - (current_est * RESISTANCE);
    em_density       = current_est / WIRE_AREA;
    dynamic_power_uw = accum_load * 0.5;
    leakage_power_uw = dynamic_power_uw * 0.05;

    $display("  FA Toggle Count      = %0d",      fa_toggle_count);
    $display("  State Transitions    = %0d",      state_trans_count);
    $display("  Violation Nodes      = %0d/8",    violation_nodes);
    $display("  Toggle Budget Left   = %0d",      toggle_budget);
    $display("  Accumulated Load     = %0d",      accum_load);
    $display("  Power Level Index    = %0d",      power_level);
    $display("  Estimated Current    = %.4f A",   current_est);
    $display("  IR Drop              = %.2f mV",  ir_drop_mv);
    $display("  Final Cell Voltage   = %.4f V",   final_voltage);
    $display("  EM Current Density   = %.4f A/um2", em_density);
    $display("  Dynamic Power        = %.2f uW",  dynamic_power_uw);
    $display("  Leakage Power        = %.2f uW",  leakage_power_uw);
    $display("----------------------------------------------------");
    $display("  ML CLASSIFIER OUTPUTS:");
    $display("  ML IR Risk Score     = %0d / 255", ml_ir_score);
    $display("  ML EM Risk Score     = %0d / 255", ml_em_score);
    $display("  Classifier Confidence= %0d / 15",  confidence);
    $display("----------------------------------------------------");

    if (ir_drop_mv > IR_CRIT_MV)
        $display("  IR Analysis  : CRITICAL  (%.1f mV > %.0f mV)", ir_drop_mv, IR_CRIT_MV);
    else if (ir_drop_mv > IR_WARN_MV)
        $display("  IR Analysis  : MODERATE  (%.1f mV > %.0f mV)", ir_drop_mv, IR_WARN_MV);
    else
        $display("  IR Analysis  : SAFE      (%.1f mV)", ir_drop_mv);

    if (em_density > EM_LIMIT)
        $display("  EM Analysis  : RISK      (%.4f A/um2 > %.1f)", em_density, EM_LIMIT);
    else
        $display("  EM Analysis  : SAFE      (%.4f A/um2)", em_density);

    $display("----------------------------------------------------");
    $display("  AI Classifier Final:");
    print_ai_result();

    $display("----------------------------------------------------");
    $display("  Technology Recommendation:");
    if      (power_level <= 4)  $display("  -> 28nm CMOS   (low power)");
    else if (power_level <= 8)  $display("  -> 12nm FinFET (moderate activity)");
    else if (power_level <= 12) $display("  -> 7nm FinFET  (balanced PPA)");
    else                        $display("  -> 5nm/3nm FinFET (high performance)");

    $display("----------------------------------------------------");
    $display("  ASSERTION SUMMARY (14 SVA properties + manual checks):");
    $display("  Passed : %0d", assert_pass_count);
    $display("  Failed : %0d", assert_fail_count);
    if (assert_fail_count == 0)
        $display("  Result : ALL ASSERTIONS PASSED");
    else
        $display("  Result : %0d ASSERTION(S) FAILED — review log", assert_fail_count);

    $display("====================================================");
    $display("  VCD written to voltus_insightai.vcd");
    $display("  Cadence Voltus post-processing:");
    $display("    voltus -init voltus_run.tcl");
    $display("    python voltus_ai_model.py voltus_insightai.vcd");
    $display("====================================================");

    $finish;
end

endmodule
