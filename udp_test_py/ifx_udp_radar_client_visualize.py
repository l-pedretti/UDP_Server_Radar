#******************************************************************************
# File Name:   udp_client.py
#
# Description: A simple python based UDP client.
# 
#********************************************************************************
# Copyright 2020-2022, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#********************************************************************************

#!/usr/bin/env python
import os
import socket
import optparse
import time
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.abspath(''), '..')))  # nopep8
import matplotlib.pyplot as plt
import numpy as np

from datetime import datetime

# datetime object containing current date and time
now = datetime.now()

BUFFER_SIZE = 1024+8

# IP details for the UDP server
DEFAULT_IP   = '192.168.1.36'  # IP address of the UDP server
DEFAULT_PORT = 57345             # Port of the UDP server for data
DEFAULT_MODE = "data"

received_packets = {} # Dictionary to collect received packets
total_packets = 12 # Total number of packets per frame 

# Function to reconstruct the entire frame
def reconstruct_frame(packets): 
        frame_data = b''.join(packets) 
        dt = np.dtype(np.float32) # radar data split into bytes combine them 
        dt = dt.newbyteorder('L') # Little endian 
        radar_data = np.frombuffer(frame_data, dtype=dt) 
        return radar_data

def udp_client_radar_test(server_ip, server_port):
        """
         server_ip: IP address of the udp server
         server_port: port on which the server is listening

        This functions intializes the connection to udp server and starts radar device in test
        mode. The status is shown on the terminal.
        
        """
        print("================================================================================")
        print("UDP Client for Radar data test")
        print("================================================================================")
        print("Sending radar configuration. IP Address:",server_ip, " Port:",server_port)
        
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #s.bind((server_ip, server_port))

         
        print("Start radar device in test mode")
        s.sendto('{"radar_transmission":"test"}'.encode(), (server_ip, server_port))
        
        while True:
                try:
                        msg, adr  = s.recvfrom(BUFFER_SIZE);
                        print(msg.decode())
                except KeyboardInterrupt:
                        break

def udp_client_radar( server_ip, server_port):
        """
         server_ip: IP address of the udp server
         server_port: port on which the server is listening

        This functions intializes the connection to udp server and starts radar device with
        given configuration. The radar raw data is read from the socket and frame number is
        shown on the terminal. 
        """
        
    
        print("================================================================================")
        print("UDP Client for Radar data")
        print("================================================================================")
        print("Sending radar configuration. IP Address:",server_ip, " Port:",server_port)

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #s.bind((server_ip, server_port))

        # radar data tranmission mode with presence application settings
        print("Start radar device with data tranmission enabled")
        s.sendto('{"radar_transmission":"enable"}'.encode(), (server_ip, server_port))
        
        dt_string = now.strftime("%d_%m_%Y-%H_%M_%S")
        path = "saved_data/" + dt_string
        isExist = os.path.exists(path)
        if not isExist:
                os.makedirs(path)

        counter = 0
        img = None
        while True:
                try:
                        data, adr  = s.recvfrom(BUFFER_SIZE)
                        frame_num = int.from_bytes(data[2:6], 'little')
                        packet_num = int.from_bytes(data[6:8], 'little')
                        #print(f"Received packet {packet_num} of frame {frame_num}")

                        if frame_num not in received_packets: 
                                received_packets[frame_num] = [None] * total_packets 

                        received_packets[frame_num][packet_num] = data[8:] # Save packet data 

                        # Check if all the packets of the frame have been received 
                        if all(received_packets[frame_num]): 
                                radar_data = reconstruct_frame(received_packets[frame_num]) 
                                print(f"Received complete frame {frame_num}")
                                print("Received data size: ", radar_data.shape)
                                radar_data = radar_data.reshape((3, 32, 32))
                                #print(radar_data)
                                if img is None:
                                        img = plt.imshow(np.moveaxis(radar_data[None, :][0], 0, -1))
                                else:
                                        img.set_data(np.moveaxis(radar_data[None, :][0], 0, -1))
                                plt.pause(0.1)
                                plt.draw()
                                #np.save(path+"/"+str(counter), radar_data)
                                counter += 1
                except KeyboardInterrupt:
                        break

        
if __name__ == '__main__':
        parser = optparse.OptionParser()
        parser.add_option("-p", "--port", dest="port", type="int", default=DEFAULT_PORT, help="Port to listen on [default: %default].")
        parser.add_option("--hostname", dest="hostname", default=DEFAULT_IP, help="Hostname or IP address of the server to connect to.")
        parser.add_option("-m", "--mode", dest="mode", type="string", default=DEFAULT_MODE, help="Mode for radar: test, data.")
        (options, args) = parser.parse_args()
        #start udp client to connect to radar device

        if options.mode == "test":
                udp_client_radar_test(options.hostname, options.port)
        else:
                udp_client_radar(options.hostname, options.port)    

