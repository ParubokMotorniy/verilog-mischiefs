(* top *) module main #(
    parameter CLK_FREQ   = 106560,
    parameter CPOL       = 1'b0,
    parameter CPHA       = 1'b0,
    parameter WIDTH      = 16,    
    parameter LSB        = 1'b0,
    parameter BAUD_RATE  = 8
)(
	input wire [3:0] x_coord,  // Joystick X input: 0 - 15
    input wire [3:0] y_coord,  // Joystick Y input: 0 - 15

    (* iopad_external_pin, clkbuf_inhibit *) input wire i_clk_m,
    (*iopad_external_pin*) input wire i_rst_m,

    // SPI interface to MAX7221
    (*iopad_external_pin*) output  o_mosi_m,
    (*iopad_external_pin*) output  o_sck_m,
    (*iopad_external_pin*) output  o_ss_n_m,

    //Interface to bram
    (*iopad_external_pin*) output bram_en,
    (*iopad_external_pin*) output [1:0] bram_bus_width,
    (*iopad_external_pin*) output bram_rd_clk_inv,

    (*iopad_external_pin*) output  bram_rd_clk_en,
    (*iopad_external_pin*) output  bram_rd_en,

    (*iopad_external_pin*) output  [8:0] bram_addr_bus,
    (*iopad_external_pin*) input   [7:0] bram_rd_bus,
    
    // control
	(*iopad_external_pin*) output  [7:0] stick_output_enable,
    (*iopad_external_pin*) output  [3:0] matrix_output_enable,
    (*iopad_external_pin*) output  clk_enable,
    (*iopad_external_pin*) output  clk_mode,
    (*iopad_external_pin*) output  rst_input_enable,
    (*iopad_external_pin*) output  [2:0] postdiv_mode,
    (*iopad_external_pin*) output  postdiv_enable
);
    // IO initialization
	assign stick_output_enable = 8'h00; //stick gpios
	assign matrix_output_enable = 4'hE; //matrix gpios
	
	assign clk_enable = 1;
	assign clk_mode = 0;
	
	assign rst_input_enable = 0;
	
	assign postdiv_enable = 1;
	assign postdiv_mode = 3'b101;
	
    // Constants
    localparam signed [5:0] CENTER = 4'd7;
    localparam signed [5:0] DEADZONE = 4'd1;

    localparam signed [15:0] MAX_SPEED = 25;

    localparam [3:0] RES_POW = 10;
    localparam signed [15:0] RESOLUTION = 1 << RES_POW;

    localparam signed [15:0] MAX_POS = 8000;
    localparam signed [15:0] UPDATE_STEP = 75;

    localparam [3:0] LOGO_DURATION = 4'd5;
    localparam [4:0] SPI_FIFO_DEPTH = 12;
    localparam [4:0] SNAKE_LENGTH = 5;

    // SPI master instance
    reg data_fed = 1'b0;
    reg slave_select = 1'b0;

    reg [WIDTH-1:0] spi_tx_data;

    wire o_tx_int;
    wire o_rx_int;

    spi_master #(
        .CPOL(CPOL),
        .CPHA(CPHA),
        .WIDTH(WIDTH),
        .LSB(LSB),
        .BAUD_RATE(BAUD_RATE)
    ) spi_inst (
        //common
        .i_clk(i_clk_m),
        .i_rst(i_rst_m),
        
        //driver interface
        .o_mosi(o_mosi_m),
        .o_sck(o_sck_m),
        .o_ss_n(o_ss_n_m),
        
        //spi module control
        .i_reset(i_rst_m),
        .i_ss_n_en(slave_select),
        .i_tx_data_valid(data_fed),
        
        //write
        .i_tx_data(spi_tx_data),
        .o_tx_int(o_tx_int),
        
        //read
        .o_rx_int(o_rx_int)
    );

    //spi writing loop
    reg [15:0] spi_fifo [0:SPI_FIFO_DEPTH - 1];
    reg [4:0] spi_fifo_in = 0; //points one above the actual data
    reg [4:0] spi_fifo_out = 0; //points at the slot being processed
    reg [4:0] spi_write_state = 0;
    wire [4:0] spi_fifo_slots_available =  spi_fifo_in < spi_fifo_out ? (spi_fifo_out - spi_fifo_in - 1) : (SPI_FIFO_DEPTH - 1) - (spi_fifo_in - spi_fifo_out);

    task push_data_to_spi_fifo(input [15:0] data);
        begin
            spi_fifo[spi_fifo_in] <= data;                    
            spi_fifo_in <= (spi_fifo_in + 1) % SPI_FIFO_DEPTH;
        end
    endtask                  

    always @(posedge i_clk_m) begin
        if(i_rst_m) begin
            spi_write_state <= 0;
            spi_fifo_out <= 0;
            spi_tx_data <= 16'hFCF1;
            data_fed  <= 0;
            slave_select <= 1'b0;
        end else if(spi_fifo_in != spi_fifo_out) begin
            case (spi_write_state)               
                0: begin                   
                    spi_tx_data <= spi_fifo[spi_fifo_out]; 
                    data_fed <= 1'b1;      
                    spi_write_state <= 1;    
                end                        
                1 : begin                  
                    slave_select <= 1'b1;  
                    spi_write_state <= 2;    
                end                        
                2: begin                   
                    if(o_rx_int) begin     
                        slave_select <= 0; 
                        data_fed <= 0;     
                        spi_write_state <= 3;
                    end                    
                end              
                3: begin
                    spi_write_state <= 0;
                    spi_fifo_out <= (spi_fifo_out + 1) % SPI_FIFO_DEPTH;
                end
            endcase                        
        end
    end

    //bram reading loop
    reg [7:0] bram_byte = 0;
    reg [2:0] bram_read_state = 0;
    reg [3:0] bram_row_counter = 1;
    reg [8:0] bram_addr = 0;
    reg bram_read_pending = 0;
    assign bram_en = 1'b1; //enable bram
    assign bram_bus_width = 2'b00;  //read by bytes
    assign bram_rd_clk_inv = 1'b0; //read on rising edge

    always @(negedge i_clk_m) begin
        if(i_rst_m) begin
            bram_read_state <= 0;
            bram_rd_clk_en <= 0;
            bram_rd_en <= 0;
        end else begin 
            if(bram_read_pending == 1) begin
                if(bram_read_state < 4) begin             
                    case (bram_read_state)                            
                        0: begin                                
                            bram_rd_clk_en <= 0;                
                            bram_rd_en <= 0;                                 
                            bram_addr_bus <= bram_addr;          
                        end                                    
                        1: begin                               
                            bram_rd_clk_en <= 1;               
                            bram_rd_en <= 1;                                   
                        end                                    
                        2: begin                               
                            bram_byte <= bram_rd_bus;                        
                        end           
                        3: begin
                            bram_rd_clk_en <= 0;                
                            bram_rd_en <= 0; 
                        end
                    endcase                                       
                    bram_read_state <= bram_read_state + 1;   
                end
            end else begin 
                bram_read_state <= 0;
            end
        end
    end

    //dot state update loop
    wire signed [5:0] delta_x = $signed({2'b00, x_coord}) - CENTER;
    wire signed [5:0] delta_y = $signed({2'b00, y_coord}) - CENTER;

    wire signed [15:0] velocity_x = (delta_x > DEADZONE || delta_x < -DEADZONE) ? ((delta_x * MAX_SPEED) / CENTER) : 0;
    wire signed [15:0] velocity_y = (delta_y > DEADZONE || delta_y < -DEADZONE) ? ((delta_y * MAX_SPEED) / CENTER) : 0;

    wire signed [15:0] pos_delta_x = (UPDATE_STEP  * RESOLUTION * velocity_x) / CLK_FREQ;
    wire signed [15:0] pos_delta_y = (UPDATE_STEP  * RESOLUTION * velocity_y) / CLK_FREQ;

    reg [15:0] latent_dot_x = 4 << RES_POW;
    reg [15:0] latent_dot_y = 5 << RES_POW;

    reg [31:0] update_timer = 0;

    wire [3:0] mapped_dot_x = latent_dot_x >> RES_POW;
    wire [3:0] mapped_dot_y = 16'd8 - (latent_dot_y >> RES_POW);

    reg [3:0] mapped_prev_dot_x [0:SNAKE_LENGTH - 1];
    reg [3:0] mapped_prev_dot_y [0:SNAKE_LENGTH - 1];

    reg [SNAKE_LENGTH - 1:0] snake_head_ptr = 0;
    reg [SNAKE_LENGTH - 1:0] snake_tail_ptr = 0;

    always @(posedge i_clk_m) begin
        if(i_rst_m) begin
            latent_dot_x <= 4 << RES_POW;
            latent_dot_y <= 5 << RES_POW;

            update_timer <= 0;
        end else begin
            if(update_timer == UPDATE_STEP) begin
                //change x position based on velocity and time delta
                if(velocity_x > 0) latent_dot_x <= (latent_dot_x + pos_delta_x) % MAX_POS;
                else if(velocity_x < 0) latent_dot_x <= (latent_dot_x > -pos_delta_x) ? (latent_dot_x + pos_delta_x) : ((MAX_POS + pos_delta_x) + latent_dot_x);
                
                //change y position based on velocity and time delta
                if(velocity_y > 0) latent_dot_y <= (latent_dot_y + pos_delta_y) % MAX_POS;
                else if(velocity_y < 0) latent_dot_y <= (latent_dot_y > -pos_delta_y) ? (latent_dot_y + pos_delta_y) : ((MAX_POS + pos_delta_y) + latent_dot_y);
                
                update_timer <= 0;
            end else update_timer <= update_timer + 1;
        end
    end

    reg [2:0] draw_state = 0;
    reg [2:0] logo_state = 0;
    reg [31:0] logo_counter = LOGO_DURATION * CLK_FREQ;
    integer g = 0;

    function [7:0] accumulate_row_dots(input [3:0] equality_value);
        accumulate_row_dots = 0;
        for(g = 0; g < SNAKE_LENGTH; g = g + 1) begin
            if(equality_value == mapped_prev_dot_y[g]) begin
                accumulate_row_dots = accumulate_row_dots | (8'h01 << mapped_prev_dot_x[g]);
            end
        end
    endfunction

    always @(posedge i_clk_m) begin
        if (i_rst_m) begin
            snake_tail_ptr <= 0;
            snake_head_ptr <= SNAKE_LENGTH - 1;

            bram_row_counter <= 1;
            bram_read_pending <= 0;
            logo_counter <= LOGO_DURATION * CLK_FREQ;
            
            logo_state <= 0;
            draw_state <= 0;

            //pre-allocated packets for matrix initialization 
            spi_fifo[0] <= 16'hFCF1;
            spi_fifo[1] <= 16'hFBFF;
            spi_fifo[2] <= 16'hFAFF;
            spi_fifo[3] <= 16'hF900;
            spi_fifo_in <= 4;
        end 
        else if(bram_row_counter < 9) begin
            case (logo_state)
            0: begin
                if(bram_read_state < 4) begin
                    bram_addr <= bram_row_counter - 1; //keeps incrementally reading stuff
                    bram_read_pending <= 1;
                end else logo_state <= 1;
            end
            1: begin
                if(spi_fifo_slots_available > 0) begin
                    push_data_to_spi_fifo({4'h0, bram_row_counter, bram_byte});
                    logo_state <= 2;
                end
            end
            2: begin
                bram_read_pending <= 0;
                bram_row_counter <= bram_row_counter + 1;
                logo_state <= 0;
            end
            endcase
        end
        else if(logo_counter > 0) begin
            logo_counter <= logo_counter - 1;
        end 
        else begin 
            case (draw_state)
                0: begin
                    if(mapped_dot_x != mapped_prev_dot_x[snake_head_ptr] || mapped_dot_y != mapped_prev_dot_y[snake_head_ptr]) begin
                        if(spi_fifo_slots_available > 0) begin
                            //erases the outdated tail
                            push_data_to_spi_fifo({4'h0, mapped_prev_dot_y[snake_tail_ptr], accumulate_row_dots(mapped_prev_dot_y[snake_tail_ptr]) & ~(8'h01 << mapped_prev_dot_x[snake_tail_ptr])});

                            draw_state <= 1;
                        end
                    end
                end
                1: begin 
                    if(spi_fifo_slots_available > 0) begin

                        snake_head_ptr = (snake_head_ptr + 1) % SNAKE_LENGTH;
                        snake_tail_ptr = (snake_tail_ptr + 1) % SNAKE_LENGTH;

                        mapped_prev_dot_x[snake_head_ptr] = mapped_dot_x; //puts the new head to the array
                        mapped_prev_dot_y[snake_head_ptr] = mapped_dot_y;

                        push_data_to_spi_fifo({4'h0, mapped_dot_y, accumulate_row_dots(mapped_dot_y)}); //we only have to redraw the head

                        draw_state <= 0;
                    end
                end
            endcase
        end
    end
endmodule

