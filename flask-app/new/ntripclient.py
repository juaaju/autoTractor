#!/usr/bin/env python3
"""
NTRIP Client based on jcmb/NTRIP
Modified for Python 3 and Indonesian BIG server

Usage:
python3 ntrip_client.py -u toor -p toor123456 -t -6.1167 -g 106.8167 -v nrtk.big.go.id 2001 vrs-rtcm3
"""

import socket
import sys
import datetime
import base64
import time
import os
from optparse import OptionParser

version = 0.3
useragent = "NTRIP JCMBsoftPythonClient/%.1f" % version

# reconnect parameter (fixed values):
factor = 2  # How much the sleep time increases with each failed attempt
maxReconnect = 1
maxReconnectTime = 1200
sleepTime = 1  # So the first one is 1 second

class NtripClient(object):
    def __init__(self,
                 buffer=50,
                 user="",
                 out=sys.stdout,
                 port=2001,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=46,
                 lon=122,
                 height=1212,
                 ssl=False,
                 verbose=False,
                 UDP_Port=None,
                 V2=False,
                 headerFile=sys.stderr,
                 headerOutput=False,
                 maxConnectTime=0
                 ):
        self.buffer = buffer
        self.user = base64.b64encode(user.encode('ascii')).decode('ascii')
        self.out = out
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.setPosition(lat, lon)
        self.height = height
        self.ssl = ssl
        self.host = host
        self.verbose = verbose
        self.UDP_Port = UDP_Port
        self.V2 = V2
        self.headerFile = headerFile
        self.headerOutput = headerOutput
        self.maxConnectTime = maxConnectTime
        
        if verbose:
            print("NTRIP Client initialized")
            print(f"Server: {caster}:{port}")
            print(f"Mountpoint: {mountpoint}")
            print(f"Position: {lat:.4f}, {lon:.4f}, {height}m")

    def setPosition(self, lat, lon):
        self.flagN = "N"
        self.flagE = "E"
        if lon > 180:
            lon = (lon - 360) * -1
            self.flagE = "W"
        elif (lon < 0 and lon >= -180):
            lon = lon * -1
            self.flagE = "W"
        elif lon < -180:
            lon = lon + 360
            self.flagE = "E"
        else:
            self.flagE = "E"

        if lat < 0:
            lat = lat * -1
            self.flagN = "S"
        
        self.lonDeg = int(lon)
        self.latDeg = int(lat)
        self.lonMin = (lon - self.lonDeg) * 60
        self.latMin = (lat - self.latDeg) * 60

    def getMountPointString(self):
        mountPointString = "GET %s HTTP/1.1\r\n" % self.mountpoint
        mountPointString += "User-Agent: %s\r\n" % useragent
        mountPointString += "Accept: */*\r\n"
        mountPointString += "Authorization: Basic %s\r\n" % self.user
        if self.host or self.V2:
            mountPointString += "Host: %s:%i\r\n" % (self.caster, self.port)
        if self.V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "Connection: close\r\n"
        mountPointString += "\r\n"
        return mountPointString

    def getGGAString(self):
        now = datetime.datetime.utcnow()
        ggaString = "GPGGA,%02d%02d%04.1f,%02d%08.5f,%1s,%03d%08.5f,%1s,1,08,0.9,%.1f,M,0.0,M,," % \
            (now.hour, now.minute, now.second, self.latDeg, self.latMin, self.flagN, self.lonDeg, self.lonMin, self.flagE, self.height)
        
        checksum = 0
        for char in ggaString:
            checksum ^= ord(char)
        
        return "$%s*%02X\r\n" % (ggaString, checksum)

    def readLoop(self):
        reconnectTry = 1
        sleepTime = 1
        
        while reconnectTry <= maxReconnect:
            found_header = False
            if self.verbose:
                print(f"Connection attempt {reconnectTry}")
            
            try:
                if self.ssl:
                    import ssl
                    context = ssl.create_default_context()
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket = context.wrap_socket(sock, server_hostname=self.caster)
                else:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
                self.socket.settimeout(10)
                error_indicator = 1
                self.socket.connect((self.caster, self.port))
                error_indicator = 2
                
                if self.verbose:
                    print("Sending NTRIP request...")
                    
                self.socket.send(self.getMountPointString().encode())
                error_indicator = 3
                
                data = self.socket.recv(self.buffer).decode('ascii')
                if self.verbose:
                    print("Response header:")
                    print(data)
                
                if self.headerOutput:
                    self.headerFile.write(data)
                
                if "200 OK" in data:
                    found_header = True
                    if self.verbose:
                        print("Connection successful, starting data stream...")
                elif "401" in data:
                    print("Authentication failed (401) - check username/password")
                    return False
                elif "404" in data:
                    print("Mountpoint not found (404)")
                    return False
                else:
                    print("Unexpected response from server")
                    print(data)
                    return False
                
                if found_header:
                    if self.verbose:
                        print("Sending initial GGA...")
                    self.socket.send(self.getGGAString().encode())
                    
                    # Start data reception loop
                    self.receiveData()
                    
            except socket.timeout:
                print("Connection timeout")
                if self.socket:
                    self.socket.close()
                    
            except Exception as e:
                print(f"Connection error: {e}")
                if self.socket:
                    self.socket.close()
                    
            if reconnectTry < maxReconnect:
                print(f"Retrying in {sleepTime} seconds...")
                time.sleep(sleepTime)
                sleepTime *= factor
                if sleepTime > maxReconnectTime:
                    sleepTime = maxReconnectTime
                    
            reconnectTry += 1
            
        print("Max reconnection attempts reached")
        return False

    def receiveData(self):
        """Main data reception loop"""
        lastGGATime = time.time()
        rtcm_count = 0
        
        try:
            while True:
                # Send GGA every 10 seconds
                current_time = time.time()
                if current_time - lastGGATime > 10:
                    self.socket.send(self.getGGAString().encode())
                    lastGGATime = current_time
                    if self.verbose:
                        print("Sent GGA heartbeat")
                
                # Check for incoming data
                self.socket.settimeout(1.0)
                try:
                    data = self.socket.recv(self.buffer)
                    if not data:
                        print("Server closed connection")
                        break
                        
                    rtcm_count += 1
                    if self.verbose:
                        print(f"Received RTCM data #{rtcm_count}: {len(data)} bytes")
                        if rtcm_count <= 3:  # Show hex for first 3 packets
                            hex_data = ' '.join(f'{b:02x}' for b in data[:20])
                            print(f"  Hex (first 20 bytes): {hex_data}")
                    
                    # Write data to output
                    if hasattr(self.out, 'buffer'):
                        self.out.buffer.write(data)
                        self.out.buffer.flush()
                    else:
                        self.out.write(data)
                        if hasattr(self.out, 'flush'):
                            self.out.flush()
                    
                    # UDP broadcast if configured
                    if self.UDP_Port:
                        self.broadcastUDP(data)
                        
                except socket.timeout:
                    # Normal timeout, continue
                    continue
                    
        except KeyboardInterrupt:
            print("Interrupted by user")
        except Exception as e:
            print(f"Data reception error: {e}")
        finally:
            if self.socket:
                self.socket.close()

    def broadcastUDP(self, data):
        """Broadcast RTCM data via UDP"""
        try:
            if not hasattr(self, 'udp_socket'):
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            self.udp_socket.sendto(data, ('255.255.255.255', self.UDP_Port))
        except Exception as e:
            if self.verbose:
                print(f"UDP broadcast error: {e}")

if __name__ == '__main__':
    usage = "NtripClient.py [options] caster port mountpoint"
    parser = OptionParser(version=str(version), usage=usage)
    
    parser.add_option("-u", "--user", type="string", dest="user", default="IBS", 
                      help="The Ntripcaster username. Default: %default")
    parser.add_option("-p", "--password", type="string", dest="password", default="IBS", 
                      help="The Ntripcaster password. Default: %default")
    parser.add_option("-t", "--latitude", type="float", dest="lat", default=-6.1167, 
                      help="Your latitude in decimal degrees. Default: %default (Jakarta)")
    parser.add_option("-g", "--longitude", type="float", dest="lon", default=106.8167, 
                      help="Your longitude in decimal degrees. Default: %default (Jakarta)")
    parser.add_option("-e", "--height", type="float", dest="height", default=10.0, 
                      help="Your ellipsoid height in meters. Default: %default")
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=False, 
                      help="Verbose output")
    parser.add_option("-s", "--ssl", action="store_true", dest="ssl", default=False, 
                      help="Use SSL for the connection")
    parser.add_option("-H", "--host", action="store_true", dest="host", default=False, 
                      help="Include host header")
    parser.add_option("-r", "--Reconnect", type="int", dest="maxReconnect", default=1, 
                      help="Number of reconnection attempts")
    parser.add_option("-D", "--UDP", type="int", dest="UDP", default=None, 
                      help="Broadcast received data on the provided UDP port")
    parser.add_option("-2", "--V2", action="store_true", dest="V2", default=False, 
                      help="Make a NTRIP V2 Connection")
    parser.add_option("-f", "--outputFile", type="string", dest="outputFile", default=None, 
                      help="Write to this file, instead of stdout")
    parser.add_option("-m", "--maxtime", type="int", dest="maxConnectTime", default=0, 
                      help="Maximum connection time in seconds")
    parser.add_option("--Header", action="store_true", dest="headerOutput", default=False, 
                      help="Write headers to stderr")
    
    (options, args) = parser.parse_args()
    
    if len(args) != 3:
        print("Error: Need exactly 3 arguments: caster port mountpoint")
        print("Example: python3 ntrip_client.py -u toor -p toor123456 -v nrtk.big.go.id 2001 vrs-rtcm3")
        sys.exit(1)
    
    caster = args[0]
    port = int(args[1])
    mountpoint = args[2]
    
    if not mountpoint.startswith("/"):
        mountpoint = "/" + mountpoint
    
    # Setup global variables
    maxReconnect = options.maxReconnect
    
    # Create output stream
    if options.outputFile:
        try:
            output_file = open(options.outputFile, 'wb')
        except Exception as e:
            print(f"Error opening output file: {e}")
            sys.exit(1)
    else:
        output_file = sys.stdout
    
    # Create NTRIP client
    ntripArgs = {
        'user': f"{options.user}:{options.password}",
        'caster': caster,
        'port': port,
        'mountpoint': mountpoint,
        'lat': options.lat,
        'lon': options.lon,
        'height': options.height,
        'ssl': options.ssl,
        'host': options.host,
        'verbose': options.verbose,
        'UDP_Port': options.UDP,
        'V2': options.V2,
        'out': output_file,
        'headerOutput': options.headerOutput,
        'maxConnectTime': options.maxConnectTime
    }
    
    if options.verbose:
        print("=== NTRIP Client Configuration ===")
        print(f"Server: {caster}:{port}")
        print(f"Mountpoint: {mountpoint}")
        print(f"Username: {options.user}")
        print(f"Position: {options.lat:.4f}°, {options.lon:.4f}°, {options.height}m")
        print(f"NTRIP Version: {'2.0' if options.V2 else '1.0'}")
        print(f"SSL: {'Yes' if options.ssl else 'No'}")
        if options.UDP:
            print(f"UDP Broadcast: Port {options.UDP}")
        print()
    
    # Start client
    client = NtripClient(**ntripArgs)
    try:
        success = client.readLoop()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if options.outputFile and output_file != sys.stdout:
            output_file.close()