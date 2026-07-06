// Coulomb-counting SoC accumulator (Q1.31, 1.0 = 2^31), saturating at [0, 2^31]:
//   soc -= (I * K_DSOC) >>> 8            K_DSOC = dt*2^31/(Q_nom*512)*2^8
//   charging (I < 0): delta = (delta * EFF) >>> 15
//   if corr_en:       soc += (ALPHA * (soc_ocv - soc)) >>> 15
`default_nettype none

module coulomb #(
    parameter signed [31:0] K_DSOC = 32'sd11930,
    parameter signed [15:0] EFF    = 16'sd32440,   // 0.99 in Q1.15
    parameter signed [15:0] ALPHA  = 16'sd655      // 0.02 in Q1.15
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,
    input  wire signed [15:0] iq,        // Q7.9, +ve = discharge
    input  wire               corr_en,   // rested && ocv_valid && !freeze
    input  wire [31:0]        soc_ocv,   // Q1.31 from the inverse OCV lookup
    output reg  [31:0]        soc        // Q1.31 unsigned
);

  localparam [31:0] FULL = 32'h8000_0000;

  wire signed [47:0] p_raw   = iq * K_DSOC;
  wire signed [47:0] d_raw   = p_raw >>> 8;
  wire signed [47:0] d_eff   = (d_raw * EFF) >>> 15;
  wire signed [47:0] delta   = (iq < 0) ? d_eff : d_raw;

  wire signed [47:0] s_cc    = $signed({16'd0, soc}) - delta;
  wire signed [47:0] s_cc_cl = (s_cc < 0)                        ? 48'sd0 :
                               (s_cc > $signed({16'd0, FULL}))   ?
                                $signed({16'd0, FULL})           : s_cc;

  wire signed [47:0] diff    = $signed({16'd0, soc_ocv}) - s_cc_cl;
  wire signed [47:0] corr    = (ALPHA * diff) >>> 15;
  wire signed [47:0] s_bl    = s_cc_cl + corr;
  wire signed [47:0] s_bl_cl = (s_bl < 0)                        ? 48'sd0 :
                               (s_bl > $signed({16'd0, FULL}))   ?
                                $signed({16'd0, FULL})           : s_bl;

  wire signed [47:0] s_next  = corr_en ? s_bl_cl : s_cc_cl;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  soc <= FULL;          // assume full pack at power-on
    else if (en) soc <= s_next[31:0];
  end

endmodule

`default_nettype wire
