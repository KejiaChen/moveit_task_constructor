import socket
import os
import tqdm

SEPARATOR = "<SEPARATOR>"
BUFFER_SIZE = 4096 # send 4096 bytes each time step

def client_program(host, port, file_name):
    # host = socket.gethostname()  # as both code is running on same pc
    # port = 12344  # socket server port number

    client_socket = socket.socket()  # instantiate
    client_socket.connect((host, port))  # connect to the server
    filesize = os.path.getsize(file_name) 
    
    # send the name and size of the file
    client_socket.send(f"{file_name}{SEPARATOR}{filesize}".encode())
    
    # send the file
    progress = tqdm.tqdm(range(filesize), f"Sending {file_name}", unit="B", unit_scale=True, unit_divisor=1024)
    with open(file_name, "rb") as f:
        while True:
            # read the bytes from the file
            bytes_read = f.read(BUFFER_SIZE)
            if not bytes_read:
                # file transmitting is done
                break
            # we use sendall to assure transimission in 
            # busy networks
            client_socket.sendall(bytes_read)
            # update the progress bar
            progress.update(len(bytes_read))

    # data = client_socket.recv(1024).decode()  # receive response
    client_socket.close()  # close the connection


if __name__ == '__main__':
    # file  = "/home/tp2/ws_moveit/src/panda_multiple_arms/saved_trajectories/smooth_real_world_traj.txt"
    # client_program('tp2-Precision-3650-Tower', port=12345, file_name=file)
    
    # send smooth trajectory to robot
    client_socket = socket.socket()  # instantiate
    # server_host = 'tp2'
    server_host = '10.157.174.101'
    server_port = 12345
    client_socket.connect((server_host, server_port))  # connect to the server
    # send the name of the file
    smooth_file_path = 'smooth'
    client_socket.send(f"{os.path.basename(smooth_file_path)}".encode())
    # send the trajectory
    # joint_traj_data = pickle.dumps(smooth_traj.q)
    # client_socket.sendall(joint_traj_data)
    # send the stopping signal
    # client_socket.send(f"end".encode())
    client_socket.close()