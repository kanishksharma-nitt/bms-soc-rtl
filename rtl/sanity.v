// Plausibility monitor. Flags (all post-update for the current sample):
//   v_oor   : cell voltage outside [V_MIN, V_MAX]
//   t_oor   : temperature outside [T_MIN, T_MAX]
//   i_stuck : identical current reading for STUCK_N samples while
//             |I| > STUCK_I_TH (below that band a stuck reading looks like
//             genuine rest, where the OCV correction handles drift instead)
// Any flag freezes the OCV correction; Coulomb counting continues.
`default_nettype none

module sanity #(
    parameter [15:0]        V_MIN      = 16'd20480,   // 2.5 V in Q3.13
    parameter [15:0]        V_MAX      = 16'd35226,   // 4.3 V
    parameter signed [15:0] T_MIN      = -16'sd40,    // -20 C in Q7.1
    parameter signed [15:0] T_MAX      = 16'sd120,    // +60 C
    parameter signed [15:0] STUCK_I_TH = 16'sd154,
    parameter [7:0]         STUCK_N    = 8'd200       // 2 s at 100 Hz
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,
    input  wire signed [15:0] iq,
    input  wire [15:0]        vq,
    input  wire signed [15:0] tq,
    output wire               v_oor,
    output wire               t_oor,
    output wire               i_stuck,
    output wire               freeze_corr
);

  reg [7:0]         run;
  reg signed [15:0] last_i;
  reg               last_valid;

  assign v_oor = (vq < V_MIN) || (vq > V_MAX);
  assign t_oor = (tq < T_MIN) || (tq > T_MAX);

  wire signed [16:0] iq17 = {iq[15], iq};
  wire signed [16:0] a_iq = iq17[16] ? -iq17 : iq17;
  wire same   = last_valid && (iq == last_i) &&
                (a_iq > {STUCK_I_TH[15], STUCK_I_TH});
  wire [7:0] run_nx = !same             ? 8'd0 :
                      (run == STUCK_N)  ? STUCK_N : run + 8'd1;

  assign i_stuck     = (run_nx >= STUCK_N);
  assign freeze_corr = v_oor || t_oor || i_stuck;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run        <= 8'd0;
      last_i     <= 16'sd0;
      last_valid <= 1'b0;
    end else if (en) begin
      run        <= run_nx;
      last_i     <= iq;
      last_valid <= 1'b1;
    end
  end

endmodule

`default_nettype wire
