#!/usr/bin/env python3
import socket
import base64

def check_mountpoints():
    """Check available mount points from NTRIP caster"""
    caster_host = 'nrtk.big.go.id'
    caster_port = 2001
    username = 'toor'  # Ganti dengan username Anda
    password = 'toor123456'  # Ganti dengan password Anda
    
    try:
        print(f"🔗 Connecting to {caster_host}:{caster_port}...")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(30)
        sock.connect((caster_host, caster_port))
        
        # Request untuk melihat sourcetable (daftar mount points)
        auth = base64.b64encode(f"{username}:{password}".encode()).decode()
        
        request = (
            f"GET / HTTP/1.1\r\n"
            f"Host: {caster_host}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP Client/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        )
        
        sock.send(request.encode())
        
        # Read response
        response = b""
        while True:
            data = sock.recv(4096)
            if not data:
                break
            response += data
        
        response_str = response.decode('utf-8', errors='ignore')
        
        print("📋 NTRIP Server Response:")
        print("=" * 60)
        print(response_str)
        print("=" * 60)
        
        # Parse mount points
        lines = response_str.split('\n')
        mount_points = []
        
        for line in lines:
            if line.startswith('STR;'):
                parts = line.split(';')
                if len(parts) > 1:
                    mount_points.append(parts[1])
        
        if mount_points:
            print(f"\n📡 Available Mount Points:")
            for mp in mount_points:
                print(f"   - {mp}")
        else:
            print("\n❌ No mount points found in response")
            
        sock.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    check_mountpoints()
