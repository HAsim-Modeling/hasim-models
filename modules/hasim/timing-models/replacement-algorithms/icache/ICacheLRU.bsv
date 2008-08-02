// Implements LRU algorithm

import Vector::*;
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

import hasim_isa::*;

`include "asim/provides/hasim_icache_types.bsh"

typedef Bit#(TLog#(`ICACHE_ASSOC)) ICACHE_LRU_BITS;
typedef Tuple2#(ICACHE_LRU_BITS, ICACHE_TAG) ICACHE_LINE;

typedef struct{
   Bool accessed_hit;
   Vector#(`ICACHE_ASSOC, Maybe#(ICACHE_LINE)) accessed_set;
   ICACHE_WAY accessed_way;
   }
ReplacementAlgorithmInput deriving (Eq, Bits);

typedef struct{
   Vector#(`ICACHE_ASSOC, Maybe#(ICACHE_LINE)) updated_set;
   ICACHE_WAY replaced_way;
   }
ReplacementAlgorithmOutput deriving (Eq, Bits);

module [HASIM_MODULE] mkICacheReplacementAlgorithm();
      
   // states
   Reg#(TOKEN) resp_tok <- mkRegU();
   Reg#(Vector#(`ICACHE_ASSOC, Maybe#(ICACHE_LINE))) resp_set <- mkRegU();
   Reg#(ICACHE_WAY) resp_way <- mkRegU();
      
   // incoming ports
   Port_Receive#(Tuple2#(TOKEN, ReplacementAlgorithmInput)) port_from_icache_core <- mkPort_Receive("icache_core_to_replacement_algorithm", 0);

   // outgoing ports
   Port_Send#(Tuple2#(TOKEN, ReplacementAlgorithmOutput)) port_to_icache_core <- mkPort_Send("replacement_algorithm_to_icache_core");  
   
   // communication with local controller
   Vector#(1, Port_Control) inports = newVector();
   Vector#(1, Port_Control) outports = newVector();
   inports[0] = port_from_icache_core.ctrl;
   outports[0] = port_to_icache_core.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   // rules
   rule handlereq;
      
     
      // read input port from icache core
      let msg_from_icache_core <- port_from_icache_core.receive();
        
      // react to input message from icache core
      case(msg_from_icache_core) matches
	 // if NoMessage
	 tagged Invalid:
	    begin
	       // send a NoMessage back to icache core and loop in the same state
	       port_to_icache_core.send(tagged Invalid);
	       
	    end
	 // if icache core is making a request
	 tagged Valid {.tok_from_icache_core, .req_from_icache_core}:
	    begin
	       
	       // check the request type
	       if (req_from_icache_core.accessed_hit)
		  // access was a hit, update LRU status bits
		  begin
		     Vector#(`ICACHE_ASSOC, Maybe#(ICACHE_LINE)) accessed_set_data = req_from_icache_core.accessed_set;
		     let accessed_way_number = req_from_icache_core.accessed_way;
		     if (accessed_set_data[accessed_way_number] matches tagged Valid {.accessed_lru, .accessed_tag})
			begin
		     
			   // loop through all the ways in the set updating LRU bits
			   for (Integer w = 0; w < `ICACHE_ASSOC ; w = w + 1)
			      begin
				 case (accessed_set_data[w]) matches
				    // update LRU bits only if cache line is valid
				    tagged Valid {.test_lru, .test_tag}: 
				       begin
					  if (test_lru < accessed_lru)
					     begin
						accessed_set_data[w] = tagged Valid tuple2(test_lru + 1, test_tag);	   
					     end
				       end
				 endcase
			      end
		  
			   // update the accessed line LRU bits
			   accessed_set_data[accessed_way_number] = tagged Valid tuple2(0, accessed_tag);
		     
			   // send respone to icache core 
			   port_to_icache_core.send(tagged Valid tuple2(tok_from_icache_core, ReplacementAlgorithmOutput{updated_set:accessed_set_data, replaced_way:accessed_way_number}));
			end
		  end	
      
	       else
		  // access was a miss, update LRU status bits and pick candidate for eviction
		  begin  
		     Vector#(`ICACHE_ASSOC, Maybe#(ICACHE_LINE)) accessed_set_data = req_from_icache_core.accessed_set;
		     ICACHE_WAY victim_way = maxBound;
		     Bool victim_found = False;
		     
		     // loop through all the ways in the set updating LRU bits
		     // loop until first invalid or the oldest way (number of cache ways - 1)
		     for (Integer w = 0; w < `ICACHE_ASSOC; w = w + 1)
			begin
			   case (accessed_set_data[w]) matches
			      // if we find a free way in the cache
			      tagged Invalid:
				 begin
				    if (!victim_found)
				       begin
					  //match {.test_lru, .test_tag} = accessed_set_data[w];
					  accessed_set_data[w] = tagged Valid tuple2(0, ?);
					  victim_way = fromInteger(w);
					  victim_found = True;
				       end
				 end
			      
			      // if there are no free ways
			      tagged Valid {.test_lru, .test_tag}:
				 begin
				    // found the oldest victim
				    if (test_lru == maxBound)
				       begin
					  if(!victim_found)
					     begin
						accessed_set_data[w] = tagged Valid tuple2(0, test_tag);
						victim_found = True;
						victim_way = fromInteger(w);
					     end
				       end
		       		    
				    
				    // victim already found
				    else
				       begin
					  accessed_set_data[w] = tagged Valid tuple2(test_lru + 1, test_tag);
				       end
				 end
			   endcase
			end
		     // send output to icache core
		     port_to_icache_core.send(tagged Valid tuple2(tok_from_icache_core, ReplacementAlgorithmOutput{updated_set:accessed_set_data, replaced_way:victim_way}));					 
		  end
	    end 
	 
      endcase
   endrule
   
endmodule

			
      
     