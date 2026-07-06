// OCV(SoC) lookup: 33-entry monotonic table (Q3.13, loaded from ocv_lut.mem)
// with linear interpolation, plus the inverse lookup SoC(OCV) for rest-time
// correction: 5-cycle binary search for the segment, then a 24-cycle
// non-restoring divide for the fraction (~31 cycles total).
`default_nettype none

module ocv_lut #(
    parameter LUT_FILE = "../rtl/ocv_lut.mem"
) (
    input  wire        clk,
    input  wire        rst_n,
    // forward lookup (combinational, debug/monitor output)
    input  wire [31:0] soc,        // Q1.31
    output wire [15:0] ocv,        // Q3.13
    // inverse lookup (multi-cycle)
    input  wire        start,
    input  wire [15:0] v_in,       // Q3.13
    output reg         done,       // 1-cycle pulse
    output reg  [31:0] soc_ocv,    // Q1.31
    output wire        busy
);

  localparam [31:0] FULL = 32'h8000_0000;

  reg [15:0] lut [0:32];
  initial $readmemh(LUT_FILE, lut);

  // ------------------------- forward: interpolate -----------------------
  wire [31:0] s_cl  = (soc >= FULL) ? FULL - 1 : soc;
  wire [5:0]  f_idx = {1'b0, s_cl[30:26]};   // 6-bit index into lut[0:32]
  wire [9:0]  f_frc = s_cl[25:16];
  wire signed [16:0] f_dif = {1'b0, lut[f_idx + 6'd1]} - {1'b0, lut[f_idx]};
  wire signed [27:0] f_stp = (f_dif * {1'b0, f_frc}) >>> 10;
  assign ocv = lut[f_idx] + f_stp[15:0];

  // ------------------------- inverse: search + divide -------------------
  localparam S_IDLE = 3'd0, S_SEARCH = 3'd1, S_DIV = 3'd2, S_OUT = 3'd3;

  reg [2:0]         state;
  reg [15:0]        v_q;
  reg [4:0]         seg, bstep;
  reg [4:0]         cnt;
  reg [23:0]        q;
  reg signed [24:0] r;
  reg [11:0]        d_q;

  wire [5:0]  probe   = {1'b0, seg} + {1'b0, bstep};
  wire        take    = (probe <= 6'd31) && (lut[probe] <= v_q);
  wire [15:0] v_diff  = v_q - lut[{1'b0, seg}];
  wire [15:0] v_step  = lut[{1'b0, seg} + 6'd1] - lut[{1'b0, seg}];
  wire signed [24:0] r_shift = {r[23:0], q[23]};
  wire signed [24:0] r_next  = r[24] ? r_shift + {13'd0, d_q}
                                     : r_shift - {13'd0, d_q};

  assign busy = (state != S_IDLE);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state   <= S_IDLE;
      v_q     <= 16'd0;
      seg     <= 5'd0;
      bstep   <= 5'd0;
      cnt     <= 5'd0;
      q       <= 24'd0;
      r       <= 25'sd0;
      d_q     <= 12'd0;
      done    <= 1'b0;
      soc_ocv <= 32'd0;
    end else begin
      done <= 1'b0;
      case (state)
        S_IDLE: begin
          if (start) begin
            if (v_in <= lut[6'd0]) begin
              soc_ocv <= 32'd0;
              done    <= 1'b1;
            end else if (v_in >= lut[32]) begin
              soc_ocv <= FULL;
              done    <= 1'b1;
            end else begin
              v_q   <= v_in;
              seg   <= 5'd0;
              bstep <= 5'd16;
              state <= S_SEARCH;
            end
          end
        end

        S_SEARCH: begin
          if (take) seg <= seg + bstep;
          if (bstep == 5'd1) begin
            state <= S_DIV;
            cnt   <= 5'd0;
            r     <= 25'sd0;
          end else begin
            bstep <= bstep >> 1;
          end
        end

        S_DIV: begin
          if (cnt == 5'd0) begin
            // latch dividend/divisor on the first DIV cycle; segment steps
            // are < 1024 counts by table construction, so (v - lo) fits 10 bits
            q   <= {4'd0, v_diff[9:0], 10'd0};       // (v-lo)<<10, 24-bit
            d_q <= {2'd0, v_step[9:0]};
            cnt <= cnt + 5'd1;
          end else if (cnt <= 5'd24) begin
            r   <= r_next;
            q   <= {q[22:0], ~r_next[24]};
            cnt <= cnt + 5'd1;
            if (cnt == 5'd24) state <= S_OUT;
          end
        end

        S_OUT: begin
          soc_ocv <= ({27'd0, seg} << 26) | ({22'd0, q[9:0]} << 16);
          done    <= 1'b1;
          state   <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
