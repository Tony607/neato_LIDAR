/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Eric Perko, Chad Rockey
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Case Western Reserve University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <xv_11_laser_driver/xv11_laser.h>

namespace xv_11_laser_driver {
	XV11Laser::XV11Laser(const std::string& port, uint32_t baud_rate, uint32_t firmware, boost::asio::io_service& io): port_(port),
	baud_rate_(baud_rate), firmware_(firmware), shutting_down_(false), serial_(io, port_) {
		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
	}
	/**checksum of the package*/
	template<std::size_t SIZE>
	void XV11Laser::checksum(std::array<uint8_t, SIZE>& onepackage){
		uint8_t i;
		uint16_t data[10];
		uint16_t checksum;
		uint32_t chk32;

		// group the data by word, little-endian
		for (i = 0; i < 10; i++) {
			data[i] = onepackage[2*i] | (((uint16_t)onepackage[2*i+1]) << 8);
		}

		// compute the checksum on 32 bits
		chk32 = 0;
		for (i = 0; i < 10; i++) {
			chk32 = (chk32 << 1) + data[i];
		}

		// return a value wrapped around on 15bits, and truncated to still fit into 15 bits
		checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ); // wrap around to fit into 15 bits
		checksum = checksum & 0x7FFF; // truncate to 15 bits
		return checksum;
	}
	//update function, takes the angle (an int, from 0 to 359) and the four bytes of data
	void XV11Laser::update_view(sensor_msgs::LaserScan::Ptr scan,uint16_t angle,uint8_t x,uint8_t x1,uint8_t x2,uint8_t x3){
		point.pos[angle] = vector( 0, 0, 0)
		pointb.pos[angle] = vector( 0, 0, 0)
		point2.pos[angle] = vector( 0, 0, 0)
		point2b.pos[angle] = vector( 0, 0, 0)
		#point3.pos[angle] = vector( 0, 0, 0)
		#point3b.pos[angle] = vector( 0, 0, 0)
		
		angle_rad = angle * pi / 180.0
		c = cos(angle_rad)
		s = sin(angle_rad)

		dist_mm = x | (( x1 & 0x1f) << 8) // data on 13 bits ? 14 bits ?
		quality = x2 | (x3 << 8) // data on 10 bits or more ?
		
		scan->ranges[angle] = dist_mm / 1000.0;
		scan->intensities[angle] = quality;
	}     
	void XV11Laser::poll(sensor_msgs::LaserScan::Ptr scan) {
		uint16_t in_frame, init_level=0, angle, index, speed_rpm;
		uint16_t nb_good = 0;
		uint16_t nb_errors = 0;
		boost::array<uint8_t, 4> raw_bytes;
		uint32_t motor_speed = 0;
		
		scan->angle_min = 0.0;
		scan->angle_max = 2.0*M_PI;
		scan->angle_increment = (2.0*M_PI/360.0);
		scan->range_min = 0.06;
		scan->range_max = 5.0;
		scan->ranges.resize(360);
		scan->intensities.resize(360);
		while(true){
			if (init_level == 0) 
			// start byte
			boost::asio::read(serial_, boost::asio::buffer(raw_bytes,1));
			b = raw_bytes[0];
			if (b == 0xFA){
				init_level = 1;
			}
			else{
				init_level = 0;
			}
			else if (init_level == 1){
				// position index 
				boost::asio::read(serial_, boost::asio::buffer(raw_bytes,1));
				b = raw_bytes[0];
				if (b >= 0xA0 and b <= 0xF9){ 
					index = b - 0xA0;
					init_level = 2;
				}
				else{
					init_level = 0;
				}
			}
			else if (init_level == 2){
				boost::array<uint8_t,2> b_speed;
				boost::array<uint8_t,4> b_data0;
				boost::array<uint8_t,4> b_data1;
				boost::array<uint8_t,4> b_data2;
				boost::array<uint8_t,4> b_data3;
				//speed	2 bytes
				boost::asio::read(serial_, boost::asio::buffer(raw_bytes,2));
				std::memcpy (b_speed.data(),raw_bytes.data(),2);
				
				// data
				boost::asio::read(serial_, boost::asio::buffer(raw_bytes,4));
				std::memcpy (b_data0.data(),raw_bytes.data(),4);
				boost::asio::read(serial_, boost::asio::buffer(raw_bytes,4));
				std::memcpy (b_data1.data(),raw_bytes.data(),4);
				boost::asio::read(serial_, boost::asio::buffer(raw_bytes,4));
				std::memcpy (b_data2.data(),raw_bytes.data(),4);
				boost::asio::read(serial_, boost::asio::buffer(raw_bytes,4));
				std::memcpy (b_data3.data(),raw_bytes.data(),4);

				// for the checksum, we need all the data of the packet...
				// this could be collected in a more elegent fashion... 
				//all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3 ;				
				boost::array<uint8_t,22> all_data;
				uint8_t copy_index = 0;
				all_data[0] = 0xFA;
				copy_index++;
				all_data[1] = index+0xA0;
				copy_index++;
				
				std::memcpy (all_data.data()+copy_index,b_speed.data(),b_speed.size());
				copy_index+=b_speed.size();
				
				std::memcpy (all_data.data()+copy_index,b_data0.data(),b_data0.size());
				copy_index+=b_data0.size();
				
				std::memcpy (all_data.data()+copy_index,b_data1.data(),b_data1.size());
				copy_index+=b_data1.size();
				
				std::memcpy (all_data.data()+copy_index,b_data2.data(),b_data2.size());
				copy_index+=b_data2.size();
				
				std::memcpy (all_data.data()+copy_index,b_data3.data(),b_data3.size());
				copy_index+=b_data3.size();				

				// checksum  
				boost::asio::read(serial_, boost::asio::buffer(raw_bytes,2));
				int incoming_checksum = (int)(raw_bytes[0]) + ((int)(raw_bytes[1]) << 8);

				// verify that the received checksum is equal to the one computed from the data
				if (XV11Laser::checksum(all_data) == incoming_checksum){
					nb_good +=1;
					motor_speed = (float)( b_speed[0] | (b_speed[1] << 8) ) / 64.0;
					
					update_view(scan, index * 4 + 0, b_data0[0], b_data0[1], b_data0[2], b_data0[3]);
					update_view(scan, index * 4 + 1, b_data1[0], b_data1[1], b_data1[2], b_data1[3]);
					update_view(scan, index * 4 + 2, b_data2[0], b_data2[1], b_data2[2], b_data2[3]);
					update_view(scan, index * 4 + 3, b_data3[0], b_data3[1], b_data3[2], b_data3[3]);
					
					scan->time_increment = motor_speed/good_sets/1e8;
				}
				else{
					// the checksum does not match, something went wrong...
					nb_errors +=1;
					
					// display the samples in an error state
					update_view(index * 4 + 0, 0, 0x80, 0, 0);
					update_view(index * 4 + 1, 0, 0x80, 0, 0);
					update_view(index * 4 + 2, 0, 0x80, 0, 0);
					update_view(index * 4 + 3, 0, 0x80, 0, 0);
				}
				init_level = 0; // reset and wait for the next packet
			}
			else{ // default, should never happen...
				init_level = 0;
			}
		}
		
	}
};
