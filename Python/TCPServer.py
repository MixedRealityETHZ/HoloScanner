import socket
import struct
import sys
import os
import numpy as np
import cv2
import time
import open3d as o3d
import pickle as pkl
from collections import defaultdict

def tcp_server():
    serverHost = '' # localhost
    serverPort = 9090
    save_folder = 'data/'

    if not os.path.isdir(save_folder):
        os.mkdir(save_folder)

    # Create a socket
    sSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind server to port
    try:
        sSock.bind((serverHost, serverPort))
        print('Server bind to port '+str(serverPort))
    except socket.error as msg:
        print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
        return

    cv2.imshow('AHaT', np.zeros((512, 512), np.uint8))
    cv2.waitKey(1)

    sSock.listen(10)
    print('Start listening...')
    sSock.settimeout(3.0)
    while True:
        try:
            conn, addr = sSock.accept() # Blocking, wait for incoming connection
            break
        except KeyboardInterrupt:
            sys.exit(0)
        except Exception:
            continue

    print('Connected with ' + addr[0] + ':' + str(addr[1]))

    data = bytes()
    frame_counts = defaultdict(int)
    last_stat_time = time.time()
    while True:
        data_length = 0  # in bytes

        # Receive at least the header and the length
        while len(data) < 5:
            buf = conn.recv(5)
            if len(buf) != 0:
                data += buf
        
        timestamp = str(round(time.time_ns() * 100))  # hundred of nanoseconds

        assert len(data) >= 5
        header = data[0:1].decode('utf-8')
        data_length = struct.unpack(">i", data[1:5])[0]  # data length
        # print(f"[{header}] {data_length}")

        frame_counts[header] += 1
        current_time = time.time()
        if current_time - last_stat_time > 1:
            # print("[Stats]", "    ".join([f"{key}: {round(value / (current_time - last_stat_time))} fps" for key, value in frame_counts.items()]))
            frame_counts.clear()
            last_stat_time = current_time
        
        # Receive at least the whole package
        while len(data) < 5 + data_length:
            buf = conn.recv(data_length)
            if len(buf) != 0:
                data += buf


        # Decode the buffer
        if header == 'd':
            # Show depth sensor image
            depth_img_np = np.frombuffer(data[5:5 + data_length], np.uint16).reshape((512,512))
            depth_img_np = depth_img_np.astype(float) / 1000
            depth_img_np = (1.0 - np.clip(depth_img_np, 0.2, 1.0)) * 255
            depth_img_np = depth_img_np.astype(np.uint8)
            cv2.imshow('AHaT', depth_img_np)
            cv2.waitKey(1)
        elif header == 'r':
            rig2world = np.frombuffer(data[5:5 + data_length], np.float32).reshape(4, 4)
            # print(rig2world)
        elif header == 'e':
            ahat_extrinsics = np.frombuffer(data[5:5 + data_length], np.float32).reshape(4, 4)
            print(f"AHAT extrinsics received:")
            print(ahat_extrinsics)
        elif header == 'l':
            lut = np.frombuffer(data[5:5 + data_length], np.float32)
            print(f"LUT received, length = {len(lut)}, saving to lut.bin:")
            open(os.path.join(save_folder, "lut.bin"), "wb").write(data[5:5 + data_length])
            print(lut)
        elif header == 'i':
            f = np.frombuffer(data[5:5 + data_length], np.float32)
            # print(f)
            
            # head_transform = data[:16].reshape(4, 4)
            # data = data[16:]
            # print(head_transform)

            left_hand_present = (f[16] == 1.0)
            right_hand_present = (f[16 + 1 + 26 * 16] == 1.0)
            print(f"Left hand present: {int(left_hand_present)}   Right hand present: {int(right_hand_present)}")

            

            # first_joint = f[:16].reshape(4, 4)
            # data = data[16:]
            # print(first_joint)

            

        # TODO: [Zikai] "f" and "p" has not been adapted, and the length is not in bytes...
        elif header == 'f':
            # save spatial camera images
            data_length = struct.unpack(">i", data[1:5])[0]
            ts_left, ts_right = struct.unpack(">qq", data[5:21])

            N = int(data_length/2)
            LF_img_np = np.frombuffer(data[21:21+N], np.uint8).reshape((480,640))
            RF_img_np = np.frombuffer(data[21+N:21+2*N], np.uint8).reshape((480,640))
            cv2.imwrite(save_folder + "/" + str(ts_left)+'_LF.tiff', LF_img_np)
            cv2.imwrite(save_folder + "/" + str(ts_right)+'_RF.tiff', RF_img_np)
            print('Image with ts %d and %d is saved' % (ts_left, ts_right))
        elif header == 'p':
            # save point cloud
            N_pointcloud = struct.unpack(">i", data[1:5])[0]
            print("Length of point cloud:" + str(N_pointcloud))
            pointcloud_np = np.frombuffer(data[5:5+N_pointcloud*3*4], np.float32).reshape((-1,3))
            
            timestamp = str(int(time.time()))
            temp_filename_pc = timestamp + '_pc.ply'
            print(pointcloud_np.shape)
            o3d_pc = o3d.geometry.PointCloud()
            o3d_pc.points = o3d.utility.Vector3dVector(pointcloud_np.astype(np.float64))
            o3d.io.write_point_cloud(save_folder + temp_filename_pc, o3d_pc, write_ascii=True)
            print('Saved  image to ' + temp_filename_pc)
    
        data = data[5 + data_length:]

    print('Closing socket...')
    sSock.close()


if __name__ == "__main__":
    tcp_server()
