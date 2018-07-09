	//
	//------------------------------------------------------------------------------
	//   Copyright 2010 Mentor Graphics Corporation
	//   All Rights Reserved Worldwide
	//
	//   Licensed under the Apache License, Version 2.0 (the
	//   "License"); you may not use this file except in
	//   compliance with the License.  You may obtain a copy of
	//   the License at
	//
	//       http://www.apache.org/licenses/LICENSE-2.0
	//
	//   Unless required by applicable law or agreed to in
	//   writing, software distributed under the License is
	//   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	//   CONDITIONS OF ANY KIND, either express or implied.  See
	//   the License for the specific language governing
	//   permissions and limitations under the License.
	//------------------------------------------------------------------------------
	//
	// This example illustrates how to implement a bidirectional driver-sequence use
	// model. It uses get_next_item(), item_done() in the driver.
	//
	// It includes a bidirectional slave DUT, and the bus transactions are reported to
	// the transcript.
	//
	`define uvm_record_field(NAME,VALUE) \
	   $add_attribute(recorder.tr_handle,VALUE,NAME);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//PACKAGE
	package bidirect_bus_pkg;

		import uvm_pkg::*;
		`include "uvm_macros.svh"

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Bus sequence item
		class bus_seq_item extends uvm_sequence_item;

			// Request fields
			rand logic[31:0] addr;
			rand logic[31:0] write_data;
			rand bit read_not_write;
			rand int delay;

			// Response fields
			bit error;
			logic[31:0] read_data;

			`uvm_object_utils(bus_seq_item)

			function new(string name = "bus_seq_item");
			  super.new(name);
			endfunction

			constraint at_least_1 { delay inside {[1:20]};}

			constraint align_32 {addr[1:0] == 0;}

			//constraint read_cyc { (read_not_write == 1) -> ( write_data == 'x);}

			//constraint write_cyc { (read_not_write == 0) -> ( read_data == 'x);}

			function void do_copy(uvm_object rhs);
			  bus_seq_item rhs_;

			  if(!$cast(rhs_, rhs)) begin
			    `uvm_error("do_copy", "cast failed, check types");
			  end
			  addr = rhs_.addr;
			  write_data = rhs_.write_data;
			  read_not_write = rhs_.read_not_write;
			  delay = rhs_.delay;
			  error = rhs_.error;
			  read_data = rhs_.read_data;
			endfunction: do_copy

			function bit do_compare(uvm_object rhs, uvm_comparer comparer);
			  bus_seq_item rhs_;

			  do_compare = $cast(rhs_, rhs) &&
			               super.do_compare(rhs, comparer) &&
			               addr		===	rhs_.addr &&
			               write_data 	=== 	rhs_.write_data &&
			               read_not_write 	===	rhs_.read_not_write &&
			              // delay === rhs_.delay &&//nem tudjuk elore
			               error 		=== 	rhs_.error &&
			               read_data 	=== 	rhs_.read_data;
			endfunction: do_compare

			function string convert2string();
			  return $sformatf("%s\n addr:\t%0h\n write_data:\t%0h\n read_not_write:\t%0b\n delay:\t%0d\n error:\t%0b\n read_data:\t%0h",
			                    super.convert2string(), addr, write_data, read_not_write, delay, error, read_data);
			endfunction: convert2string

			function void do_print(uvm_printer printer);

			  if(printer.knobs.sprint == 0) begin
			    $display(convert2string());
			  end
			  else begin
			    printer.m_string = convert2string();
			  end

			endfunction: do_print

			function void do_record(uvm_recorder recorder);
			  super.do_record(recorder);

			  `uvm_record_field("addr", addr);
			  `uvm_record_field("write_data", write_data);
			  `uvm_record_field("read_not_write", read_not_write);
			  `uvm_record_field("delay", delay);
			  `uvm_record_field("error", error);
			  `uvm_record_field("read_data", read_data);

			endfunction: do_record
		endclass: bus_seq_item
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Bidirectional DRIVER - uses:
		// get_next_item() to get the next instruction item
		// item_done() to indicate that the dirver has finished with the item
		class bidirect_bus_driver extends uvm_driver #(bus_seq_item);

			`uvm_component_utils(bidirect_bus_driver)

			bus_seq_item req;

			uvm_analysis_port #(bus_seq_item) d_item_collected_port; //analizis portokat at kelll majd gondolni

			virtual bus_if BUS;

			function new(string name = "bidirect_bus_driver", uvm_component parent );
			  super.new(name, parent);
			  d_item_collected_port = new("d_item_collected_port", this);
			endfunction

			task run_phase(uvm_phase phase);

			  // Default conditions:
			  BUS.valid <= 0;
			  BUS.rnw <= 1;
			  // Wait for reset to end
			  @(posedge BUS.resetn);
			  forever
			    begin
			      seq_item_port.get_next_item(req); // Start processing req item
			      repeat(req.delay) begin
			        @(posedge BUS.clk);
			      end
			      BUS.valid <= 1;
			      BUS.addr <= req.addr;
			      BUS.rnw <= req.read_not_write;
			      if(req.read_not_write == 0) begin
			        BUS.write_data <= req.write_data;
			      end
			      while(BUS.ready != 1) begin
			        @(posedge BUS.clk);
			      end
			      // At end of the pin level bus transaction
			      // Copy response data into the req fields:
			      if(req.read_not_write == 1) begin
			        req.read_data = BUS.read_data; // If read - copy returned read data
			      end
			      req.error = BUS.error; // Copy bus error status
			      BUS.valid <= 0; // End the pin level bus transaction
			      d_item_collected_port.write(req); 
			      seq_item_port.item_done(); // End of req item
				`uvm_info("driver side", req.convert2string(), UVM_LOW);
			    end
			endtask: run_phase
		endclass: bidirect_bus_driver
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// MONITOR:
		class mye_momnitor extends uvm_monitor;
			`uvm_component_utils(mye_momnitor)
			bus_seq_item req;
			virtual bus_if BUS;

			uvm_analysis_port #(bus_seq_item) item_collected_port; //analizis portokat at kelll majd gondolni

			uvm_analysis_port #(bus_seq_item) dut_in_tx_port; //analizis portokat at kelll majd gondolni
			uvm_analysis_port #(bus_seq_item) dut_out_tx_port; //analizis portokat at kelll majd gondolni

			function new(string name = "mye_momnitor", uvm_component parent);
			  super.new(name, parent);
			  item_collected_port = new("item_collected_port", this);
			  dut_in_tx_port = new("dut_in_tx_port", this);
			  dut_out_tx_port = new("dut_out_tx_port", this);
			endfunction

			task run_phase(uvm_phase phase);
				/* Request fields
					rand logic[31:0] addr;
					rand logic[31:0] write_data;
					rand bit read_not_write;
					rand int delay;

				 	Response fields
					bit error;
					logic[31:0] read_data;
				*/
				  forever begin
				      @(posedge BUS.ready);
				    req = bus_seq_item::type_id::create ("req", this);
				      req.error = BUS.error;
				      req.addr = BUS.addr; // get address
				     if(BUS.rnw)  begin // is it a write?
				         req.read_data = BUS.read_data;  // get data   
					 req.read_not_write = 1; // set op type
				      end else begin
				         req.write_data = BUS.write_data;  // get data
					 req.read_not_write = 0; // set op type
				      end
				 $display("Following goes to analysis port:");
				`uvm_info("monitor side", req.convert2string(), UVM_LOW);
				      item_collected_port.write(req); 				
				end			    
			endtask: run_phase
		endclass: mye_momnitor
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Bus SequenceR:
		class bidirect_bus_sequencer extends uvm_sequencer #(bus_seq_item);

			`uvm_component_utils(bidirect_bus_sequencer)

			function new(string name = "bidirect_bus_sequencer", uvm_component parent = null);
			  super.new(name, parent);
			endfunction
		endclass: bidirect_bus_sequencer
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
 
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Scoreboard:
		class mye_scoreboard extends uvm_scoreboard;
		 
			`uvm_component_utils(mye_scoreboard) //reg in the factory done

			//uvm_analysis_imp#(bus_seq_item, mye_scoreboard) item_collected_export; //eredeti
			uvm_analysis_export #(bus_seq_item)sb_export_before;
			uvm_analysis_export #(bus_seq_item)sb_export_after;

			uvm_tlm_analysis_fifo #(bus_seq_item) before_fifo;
			uvm_tlm_analysis_fifo #(bus_seq_item) after_fifo;

			 //bus_seq_item req;//eredeti
			bus_seq_item tr_before;
			bus_seq_item tr_after;


			function new(string name, uvm_component parent);//ez csak sablon? vagy igy mar ready?
			  super.new(name, parent);
			  tr_before = new("tr_before");
			  tr_after = new("tr_after");
			endfunction : new
			   
			function void build_phase(uvm_phase phase);
			  super.build_phase(phase);
			  sb_export_before = new("sb_export_before",this);
			  sb_export_after  = new("sb_export_after",this);
			  before_fifo      = new("before_fifo",this); 
			  after_fifo       = new("after_fifo",this);
			endfunction : build_phase

			function void connect_phase(uvm_phase phase);
			      sb_export_before.connect(before_fifo.analysis_export);
			      sb_export_after.connect(after_fifo.analysis_export);
			endfunction: connect_phase    

			    //ez nem tudom, hogy hova jojjon:
			    //
			    task run();
			      forever begin
			        before_fifo.get(tr_before);
			        after_fifo.get(tr_after);
				//compare();
				if(tr_before.compare(tr_after))begin
					//match++;
					`uvm_info("ITS A MATCH", $sformatf("%s does match %s", tr_after.convert2string(), tr_before.convert2string()), UVM_LOW);
				end else begin
					`uvm_error("Evaluator", $sformatf("%s does not match %s", tr_after.convert2string(), tr_before.convert2string()));
				end
			      end
			    endtask: run
		endclass : mye_scoreboard
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Bus SEQUENCE, which shows how the req object contains the result at the end
		// of the control handshake with the driver via the sequencer
		class bus_seq extends uvm_sequence #(bus_seq_item);

			`uvm_object_utils(bus_seq)

			bus_seq_item req;

			rand int limit = 40; // Controls the number of iterations

			function new(string name = "bus_seq");
			  super.new(name);
			endfunction

			task body;
			  req = bus_seq_item::type_id::create("req");

			  repeat(limit)
			    begin
			      start_item(req);
			      // The address is constrained to be within the address of the GPIO function
			      // within the DUT, The result will be a request item for a read or a write
			    assert(req.randomize() with {addr inside {[32'h0100_0000:32'h0100_001C]};});
				if(req.read_not_write == 1) begin
				req.write_data = 'x;
				end else begin
				req.read_data = 'x;	
				end

			      finish_item(req);
			      // The req handle points to the object that the driver has updated with response data
			      `uvm_info("seq_body", req.convert2string(), UVM_LOW);
				$display("##---------------------------##------------------------------##");
			    end
			endtask: body
		endclass: bus_seq
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// AGENT:
		class agent_my extends  uvm_agent;
			`uvm_component_utils(agent_my)
			function new(string name, uvm_component parent);
				super.new(name, parent);
			endfunction
			
			bus_seq_item req;
			bidirect_bus_driver m_driver;
			mye_momnitor m_monitor;
			bidirect_bus_sequencer m_sequencer;	

			uvm_analysis_port #(bus_seq_item) dut_in_tx_port; //analizis portokat at kelll majd gondolni
			uvm_analysis_port #(bus_seq_item) dut_out_tx_port; //analizis portokat at kelll majd gondolni			
			//uvm_sequencer #(bus_seq_item) sqr; //never extended
			virtual bus_if BUS;

			function void build_phase(uvm_phase phase);
				m_driver = bidirect_bus_driver::type_id::create("m_driver", this);
				//m_sequencer = bidirect_bus_sequencer::type_id::create("m_sequencer", this);
				m_sequencer = new("m_sequencer", this); //no factory
				m_monitor = mye_momnitor::type_id::create("m_monitor", this);
				dut_in_tx_port = new("dut_in_tx_port", this);
				dut_out_tx_port = new("dut_out_tx_port", this);
			endfunction : build_phase

			//inner connections: only inside of the agent
			function void connect_phase(uvm_phase phase); //VIRTUAL?
				m_driver.seq_item_port.connect(m_sequencer.seq_item_export); //DONE
				m_monitor.dut_out_tx_port.connect(this.dut_out_tx_port);
				m_monitor.dut_in_tx_port.connect(this.dut_in_tx_port);
				

				  if (!uvm_config_db #(virtual bus_if)::get(this, "", "BUS_vif", m_driver.BUS))
				    `uvm_error("connect_phase", "uvm_config_db #(virtual bus_if)::get(...) failed");
				  if (!uvm_config_db #(virtual bus_if)::get(this, "", "BUS_vif", m_monitor.BUS))          ////##### ezt adtam hozza
				    `uvm_error("connect_phase", "uvm_config_db #(virtual bus_if)::get(...) failed");      ////##### ezt adtam hozza
			endfunction
		endclass : agent_my
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// ENVIRONMENT:
		class env_my extends  uvm_agent;
			`uvm_component_utils(env_my)
			function new(string name, uvm_component parent);
				super.new(name, parent);
			endfunction
			mye_scoreboard m_scoreboard;     ////##### ezt adtam hozza
			agent_my agt;

			function void build_phase(uvm_phase phase);
					agt = agent_my::type_id::create("agt", this);
					m_scoreboard = mye_scoreboard::type_id::create("m_scoreboard", this);
			endfunction : build_phase

function void connect_phase(uvm_phase phase); //VIRTUAL?
					m_monitor.item_collected_port.connect(m_scoreboard.sb_export_after);
					m_driver.d_item_collected_port.connect(m_scoreboard.sb_export_before);
endfunction



		endclass : env_my
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// TEST class which instantiates, builds and connects the sequencer and the driver
		class bidirect_bus_test extends uvm_test;

			`uvm_component_utils(bidirect_bus_test)
			function new(string name = "bidirect_bus_test", uvm_component parent = null);
			  super.new(name, parent);
			endfunction

			env_my env;
			function void build_phase(uvm_phase phase);
				env = env_my::type_id::create("env", this);
			endfunction : build_phase

			task run_phase(uvm_phase phase);
			bus_seq test_seq;
			test_seq = bus_seq::type_id::create("test_seq");
			  phase.raise_objection(this, "Starting test_seq");
			  test_seq.start(env.agt.m_sequencer);
			  phase.drop_objection(this, "Finished test_seq");
			endtask: run_phase
		endclass: bidirect_bus_test
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	endpackage: bidirect_bus_pkg
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Interfaces for the bus and the DUT GPIO output
	interface bus_if;

		logic clk;
		logic resetn;
		logic[31:0] addr;
		logic[31:0] write_data;
		logic rnw;
		logic valid;
		logic ready;
		logic[31:0] read_data;
		logic error;
	endinterface: bus_if

	interface gpio_if;		
		logic[7:0]   gp_op_valid;        
		logic[255:0] gp_op;
		logic[255:0] gp_ip;
		logic clk;
	endinterface: gpio_if
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DUT - A semi-real GPIO interface with a scratch RAM
	//
	module bidirect_bus_slave(interface bus, interface gpio);

			logic[1:0] delay;

			always @(posedge bus.clk)
			  begin
			    if(bus.resetn == 0) begin
			      delay <= 0;
			      bus.ready <= 0;
			      gpio.gp_op <= 0;
			    end
			    if(bus.valid == 1) begin // Valid cycle
			      if(bus.rnw == 0) begin // Write
			        if(delay == 2) begin
			          bus.ready <= 1;
			          delay <= 0;
			          if(bus.addr inside{[32'h0100_0000:32'h0100_001C]}) begin // GPO range - 8 words or 255 bits
			            case(bus.addr[7:0])
			              8'h00: begin gpio.gp_op_valid[0] <= 1; gpio.gp_op[31 :0  ] <= bus.write_data; end
			              8'h04: begin gpio.gp_op_valid[1] <= 1; gpio.gp_op[63 :32 ] <= bus.write_data; end
			              8'h08: begin gpio.gp_op_valid[2] <= 1; gpio.gp_op[95 :64 ] <= bus.write_data; end
			              8'h0c: begin gpio.gp_op_valid[3] <= 1; gpio.gp_op[127:96 ] <= bus.write_data; end
			              8'h10: begin gpio.gp_op_valid[4] <= 1; gpio.gp_op[159:128] <= bus.write_data; end
			              8'h14: begin gpio.gp_op_valid[5] <= 1; gpio.gp_op[191:160] <= bus.write_data; end
			              8'h18: begin gpio.gp_op_valid[6] <= 1; gpio.gp_op[223:192] <= bus.write_data; end
			              8'h1c: begin gpio.gp_op_valid[7] <= 1; gpio.gp_op[255:224] <= bus.write_data; end
			            endcase
			            bus.error <= 0;
			          end
			          else begin
			            bus.error <= 1; // Outside valid write address range
			          end
			        end
			        else begin
			          delay <= delay + 1;
			          bus.ready <= 0;
			          gpio.gp_op_valid <= 0;
			        end
			      end
			      else begin // Read cycle
			        if(delay == 3) begin
			          bus.ready <= 1;
			          delay <= 0;
			          if(bus.addr inside{[32'h0100_0000:32'h0100_001C]}) begin // GPO range - 8 words or 255 bits
			            case(bus.addr[7:0])
			              8'h00: bus.read_data <= gpio.gp_op[31:0];
			              8'h04: bus.read_data <= gpio.gp_op[63:32];
			              8'h08: bus.read_data <= gpio.gp_op[95:64];
			              8'h0c: bus.read_data <= gpio.gp_op[127:96];
			              8'h10: bus.read_data <= gpio.gp_op[159:128];
			              8'h14: bus.read_data <= gpio.gp_op[191:160];
			              8'h18: bus.read_data <= gpio.gp_op[223:192];
			              8'h1c: bus.read_data <= gpio.gp_op[255:224];
			            endcase
			            bus.error <= 0;
			          end
			          else if(bus.addr inside{[32'h0100_0020:32'h0100_003C]}) begin // GPI range - 8 words or 255 bits - read only
			            case(bus.addr[7:0])
			              8'h20: bus.read_data <= gpio.gp_ip[31:0];
			              8'h24: bus.read_data <= gpio.gp_ip[63:32];
			              8'h28: bus.read_data <= gpio.gp_ip[95:64];
			              8'h2c: bus.read_data <= gpio.gp_ip[127:96];
			              8'h30: bus.read_data <= gpio.gp_ip[159:128];
			              8'h34: bus.read_data <= gpio.gp_ip[191:160];
			              8'h38: bus.read_data <= gpio.gp_ip[223:192];
			              8'h3c: bus.read_data <= gpio.gp_ip[255:224];
			            endcase
			            bus.error <= 0;
			          end
			          else begin
			            bus.error <= 1;
			          end
			        end
			        else begin
			          delay <= delay + 1;
			          bus.ready <= 0;
			        end
			      end
			    end
			    else begin
			      bus.ready <= 0;
			      bus.error <= 0;
			      delay <= 0;
			    end
			  end
	endmodule: bidirect_bus_slave

			// Top level test bench module
	module top_tb;
		import uvm_pkg::*;
		import bidirect_bus_pkg::*;

		bus_if BUS();
		gpio_if GPIO();
		bidirect_bus_slave DUT(.bus(BUS), .gpio(GPIO));

			// Free running clock
		initial
		  begin
		    BUS.clk = 0;
		    forever begin
		      #10 BUS.clk = ~BUS.clk;
		    end
		  end

			// Reset
		initial
		  begin
		    BUS.resetn = 0;
		    repeat(3) begin
		      @(posedge BUS.clk);
		    end
		    BUS.resetn = 1;
		  end

		// UVM start up:
		initial
		  begin
		    uvm_config_db #(virtual bus_if)::set(null, "*", "BUS_vif" , BUS);
		    run_test("bidirect_bus_test");
		  end
	endmodule: top_tb
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////