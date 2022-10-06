
module lightgun
(
   input        CLK,
   input        RESET,

   input [24:0] MOUSE,
   input        MOUSE_XY,

   input  [7:0] JOY_X,
   input  [7:0] JOY_Y,
   input [15:0] JOY,
   
   input        UP,
   input        DOWN,
   input        LEFT,
   input        RIGHT,
   input [7:0]  DPAD_AIM_SPEED,
  
   input        RELOAD,

   input        HDE,VDE,
   input        CE_PIX,
   input        H40,
   
   input        BTN_MODE,
   
   input  [7:0] SENSOR_DELAY,
   
   output [2:0] TARGET,
   output       SENSOR,
   output       BTN_A,
   output       BTN_B,
   output       BTN_C,
   output       BTN_START
);

parameter cross_sz = 8'd4;

assign TARGET  = { 2'd0, ~offscreen & draw};

reg  [9:0] lg_x, x;
reg  [8:0] lg_y, y;

wire [10:0] new_x = {lg_x[9],lg_x} + {{3{MOUSE[4]}},MOUSE[15:8]};
wire [9:0] new_y = {lg_y[8],lg_y} - {{2{MOUSE[5]}},MOUSE[23:16]};

reg [7:0] old_joy_x;
reg [7:0] old_joy_y;
reg [8:0] j_x;
reg [8:0] j_y;

reg offscreen = 0, draw = 0;
always @(posedge CLK) begin
   reg old_pix, old_hde, old_vde, old_ms;
   reg [9:0] hcnt;
   reg [8:0] vcnt;
   reg [8:0] vtotal;
   reg [15:0] hde_d;
   reg [9:0] xm,xp;
   reg [8:0] ym,yp;
   reg sensor_pend;
   reg [7:0] sensor_time;
   reg reload_pressed;
   reg [2:0] reload_pend;
   reg [2:0] reload;
   
   BTN_A <= reload ? 1'b1 : (reload_pend ? 1'b0 : (BTN_MODE ? MOUSE[0] : JOY[4]));
   if(BTN_MODE ? MOUSE[1] : JOY[5]) begin
      if(RELOAD) begin
         BTN_B <= 1'b0;
         reload_pressed <= 1'b1;
         if(!reload_pressed) begin
            reload_pend <= 3'd5;
         end
      end
      else BTN_B <= 1'b1;
   end
   else begin
      BTN_B <= 1'b0;
      reload_pressed <= 1'b0;
   end

   BTN_C <= JOY[6]; // Not mapped to mouse. Unsure if it's used in any game.
   BTN_START <= BTN_MODE ? MOUSE[2] : JOY[15];
   
   /*old_ms <= MOUSE[24];
   if(MOUSE_XY) begin
      if(old_ms ^ MOUSE[24]) begin
         if(new_x[10]) lg_x <= 0;
         else if(new_x[8] & (new_x[7] | new_x[6])) lg_x <= 320;
         else lg_x <= new_x[8:0];

         if(new_y[9]) lg_y <= 0;
         else if(new_y > vtotal) lg_y <= vtotal;
         else lg_y <= new_y[8:0];
      end
   end
   else begin*/
      if(H40) lg_x <= j_x + (j_x >> 2);
      else lg_x <= j_x;

      if(j_y < 8) lg_y <= 0;
      else if((j_y - 9'd8) > vtotal) lg_y <= vtotal;
      else lg_y <= j_y - 9'd8;
   //end

   if(CE_PIX) begin
      hde_d <= {hde_d[14:0],HDE};
      old_hde <= hde_d[15];
      if(~&hcnt) hcnt <= hcnt + 1'd1;
      if(~old_hde & ~HDE) hcnt <= 0;
      if(old_hde & ~hde_d[15]) begin
         if(~VDE) begin
            vcnt <= 0;
            if(vcnt) vtotal <= vcnt - 1'd1;
         end
         else if(~&vcnt) vcnt <= vcnt + 1'd1;
      end
      
      old_vde <= VDE;
      if(~old_vde & VDE) begin
      old_joy_x <= JOY_X;
      old_joy_y <= JOY_Y;
      if(old_joy_x != JOY_X || old_joy_y != JOY_Y) begin
        j_x <= JOY_X[7:0];
        j_y <= JOY_Y[7:0];
      end else begin
        if(LEFT) begin
          if (j_x >= DPAD_AIM_SPEED) j_x <= j_x - DPAD_AIM_SPEED;
          else j_x <= 0;
        end
        if(RIGHT) begin
          if(lg_x <= 8'd255 - DPAD_AIM_SPEED) j_x <= j_x + DPAD_AIM_SPEED;
          else j_x <= 8'd255;
        end
        if(UP) begin
          if (j_y >= DPAD_AIM_SPEED) j_y <= j_y - DPAD_AIM_SPEED;
          else j_y <= 0;
        end
        if(DOWN) begin
          if (j_y < vtotal - DPAD_AIM_SPEED) j_y <= j_y + DPAD_AIM_SPEED;
          else j_y <= vtotal;
        end
      end
      
         x  <= lg_x;
         y  <= lg_y;
         xm <= lg_x - cross_sz;
         xp <= lg_x + cross_sz;
         ym <= lg_y - cross_sz;
         yp <= lg_y + cross_sz;
         offscreen <= !lg_y[7:1] || lg_y >= (vtotal-1'd1);
         
         if(reload_pend && !reload) begin
            reload_pend <= reload_pend - 3'd1;
            if (reload_pend == 3'd1) reload <= 3'd5;
         end
         else if (reload) reload <= reload - 3'd1;
      end
      
      if(~&sensor_time) sensor_time <= sensor_time + 1'd1;
      if(sensor_pend) begin
         if (sensor_time >= (SENSOR_DELAY)) begin
            SENSOR <= (!reload_pend && !reload && !offscreen);
            sensor_pend <= 1'b0;
            sensor_time <= 8'd0;
         end
      end
      // Keep sensor active for a bit to mimic real light gun behavior.
      // Required for games that poll instead of using interrupts.
      else if(sensor_time > 64) SENSOR <= 1'b0;
   end

   if(HDE && VDE && (x == hcnt) && (y <= vcnt) && (y > vcnt - 8)) begin
      sensor_pend <= 1'b1;
      sensor_time <= 8'd0;
   end
   
   draw <= (hcnt >= xm && hcnt <= xp && y == vcnt) || 
            (vcnt >= ym && vcnt <= yp && x == hcnt);
end

endmodule