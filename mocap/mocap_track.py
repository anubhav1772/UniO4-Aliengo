import socket
import struct
import sys
import threading
import matplotlib.pyplot as plt
import os


# Connection parameters (can be moved to arguments)
MCAST_GRP = '239.255.42.99'  # Multicast IP (default for OptiTrack)
DATA_PORT = 1511              # Data port (default)
COMMAND_PORT = 1510           # Command port (default)

# Maximum NatNet packet size
MAX_PACKETSIZE = 100000

# NatNet message types
MSG_FRAMEOFDATA = 7
MSG_MODELDEF = 5
MSG_SERVERINFO = 4

class NatNetClient:
    def __init__(self, mcast_grp=MCAST_GRP, data_port=DATA_PORT):
        self.mcast_grp = mcast_grp
        self.data_port = data_port
        self.sock = None
        self.running = False
        self.thread = None
        self.xy_positions = []  # List of (x, y) tuples for all rigid bodies

    def create_multicast_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('', self.data_port))
        except Exception as e:
            print(f"Bind failed: {e}")
            sys.exit(1)
        mreq = struct.pack('4sl', socket.inet_aton(self.mcast_grp), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.setblocking(False)
        return sock

    def start(self):
        if self.running:
            return
        self.sock = self.create_multicast_socket()
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        print(f"Listening for OptiTrack NatNet data on {self.mcast_grp}:{self.data_port}")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        if self.sock:
            self.sock.close()
            self.sock = None

    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(MAX_PACKETSIZE)
                if len(data) < 4:
                    continue
                # First 2 bytes - messageId, next 2 - size
                message_id, nbytes = struct.unpack_from('<HH', data, 0)
                if message_id == MSG_FRAMEOFDATA:
                    print(f"\n--- FrameOfData packet ({len(data)} bytes) ---")
                    self.parse_frame_of_data(data[4:])
                elif message_id == MSG_SERVERINFO:
                    print("ServerInfo packet received")
                elif message_id == MSG_MODELDEF:
                    print("ModelDef packet received")
                else:
                    print(f"Other packet type: {message_id}")
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"Error in receive loop: {e}")

    def parse_frame_of_data(self, data):
        # Simple parsing: only header and rigid body count
        offset = 0
        # frame number (int32)
        frame_number, = struct.unpack_from('<i', data, offset)
        offset += 4
        print(f"Frame number: {frame_number}")
        # marker set count (int32)
        marker_set_count, = struct.unpack_from('<i', data, offset)
        offset += 4
        print(f"Marker set count: {marker_set_count}")
        # ... (full parsing can be implemented further)
        # For example, skip marker sets
        for _ in range(marker_set_count):
            # name (null-terminated string)
            name = b''
            while data[offset:offset+1] != b'\0':
                name += data[offset:offset+1]
                offset += 1
            offset += 1  # null-terminator
            # marker count
            marker_count, = struct.unpack_from('<i', data, offset)
            offset += 4
            # skip marker positions
            offset += 12 * marker_count
        # unlabeled markers count
        unlabeled_count, = struct.unpack_from('<i', data, offset)
        offset += 4
        offset += 12 * unlabeled_count
        # rigid body count
        rigid_body_count, = struct.unpack_from('<i', data, offset)
        offset += 4
        print(f"Rigid body count: {rigid_body_count}")
        for i in range(rigid_body_count):
            # id (int32)
            body_id, = struct.unpack_from('<i', data, offset)
            offset += 4
            # pos (3 floats)
            pos = struct.unpack_from('<fff', data, offset)
            offset += 12
            # ori (4 floats)
            ori = struct.unpack_from('<ffff', data, offset)
            offset += 16
            print(f"  RigidBody {i}: id={body_id}, pos={pos}, ori={ori}")
            # Save xy position
            self.xy_positions.append((pos[0], pos[1]))
            # (further parsing of errors, params, etc. can be added)

    def plot_xy_trajectory(self):
        if not self.xy_positions:
            print("No data to plot.")
            return
        
        # Найти первый свободный номер для префикса
        n = 1
        while os.path.exists(f"trajectory_{n}.png") or os.path.exists(f"trajectory_{n}.txt"):
            n += 1
        
        # Сгенерировать имена файлов с текущим префиксом
        image_filename = f"trajectory_{n}.png"
        data_filename = f"trajectory_{n}.txt"
        
        # Построение графика
        xs, ys = zip(*self.xy_positions)
        plt.figure(figsize=(8, 6))
        plt.plot(xs, ys, marker='o', linestyle='-', label='Trajectory (x, y)')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.title('Rigid Body XY Trajectory')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # Сохранение и вывод
        plt.savefig(image_filename)
        with open(data_filename, 'w') as f:
            for x, y in self.xy_positions:
                f.write(f"{x:.2f} {y:.2f}\n")
        
        print(f"Plot saved to {image_filename}")
        print(f"Data saved to {data_filename}")
        plt.show()


def main():
    client = NatNetClient()
    client.start()
    try:
        while True:
            pass  # Keep main thread alive
    except KeyboardInterrupt:
        print("\nExiting.")
        client.stop()
        client.plot_xy_trajectory()

if __name__ == '__main__':
    main() 